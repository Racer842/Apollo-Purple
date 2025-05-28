#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/random/random.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

LOG_MODULE_REGISTER(disco_ultrasonic, LOG_LEVEL_INF);

#define WIFI_SSID "Jake_iphone"
#define WIFI_PSK  "bbbooooo"

#define MQTT_BROKER_ADDR "172.20.10.11"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID "disco_ultrasonic_node"

/* MQTT Topics for Pump Control */
#define MQTT_TOPIC_PUMP_CONTROL "pump/activate"

static struct mqtt_client client_ctx;
static uint8_t rx_buffer[256], tx_buffer[256];
static struct sockaddr_storage broker;
static struct zsock_pollfd fds[1];
static int nfds;
static bool connected = false;

/************************************************************************/
/* PUMP CONTROL CONFIGURATION                                           */
/************************************************************************/

/* Pump control GPIO pin */
#define PUMP_CONTROL_PIN 2  // Using pin 6 for pump control

/* Global variables for pump control */
static bool pump_active = false;
static struct k_mutex pump_mutex;

static const struct device *trig_port = DEVICE_DT_GET(DT_NODELABEL(gpioc));
static const struct device *echo_port = DEVICE_DT_GET(DT_NODELABEL(gpioc));

/* Ultrasonic sensor pins */
#define ULTRASONIC1_TRIG_PIN 5
#define ULTRASONIC1_ECHO_PIN 4

/* Configuration */
#define ULTRASONIC_TIMEOUT_USEC 500000 /* Maximum timeout for echo pulse (25ms) */
#define MEASUREMENT_INTERVAL_MS 1000   /* Time between measurements in ms */
#define SOUND_SPEED_MPS 343           /* Speed of sound in m/s at room temp */

/* Global variables for ultrasonic */
static struct gpio_callback echo1_cb_data;
static uint32_t echo1_start_time;
static uint32_t echo1_pulse_duration;
static volatile bool echo1_received = false;
static uint16_t distance1_mm = 0;
static struct k_mutex distance_mutex;

/************************************************************************/
/* BLUETOOTH LE IBEACON CONFIGURATION                                   */
/************************************************************************/

#define IBEACON_PREFIX_LEN 9
#define IBEACON_UUID_LEN 16
#define IBEACON_PAYLOAD_LEN 30

static const uint8_t beacon_uuid[16] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF2, 0xF2,
};

/* Reusable function to update and advertise distance as iBeacon */
static void advertise_distance(uint16_t distance_mm)
{
    uint8_t adv_data[IBEACON_PAYLOAD_LEN] = {
        0x02, 0x01, 0x06,                         // Flags
        0x1A, 0xFF,                               // Length, Manufacturer Specific
        0x4C, 0x00,                               // Apple Company ID
        0x02, 0x15                                // iBeacon type and length
    };

    memcpy(&adv_data[9], beacon_uuid, IBEACON_UUID_LEN);

    adv_data[25] = (distance_mm >> 8) & 0xFF;     // Major (high byte)
    adv_data[26] = distance_mm & 0xFF;            // Major (low byte)

    adv_data[27] = 0x00;                          // Minor high byte (set to 0)
    adv_data[28] = 0x01;                          // Minor low byte

    adv_data[29] = 0xC5;                          // Measured Power

    struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
        BT_DATA(BT_DATA_MANUFACTURER_DATA, &adv_data[5], 25)
    };

    struct bt_le_adv_param adv_param = {
        .id = BT_ID_DEFAULT,
        .sid = 0,
        .secondary_max_skip = 0,
        .options = BT_LE_ADV_OPT_USE_NAME,
        .interval_min = 0x00A0,
        .interval_max = 0x00A0,
        .peer = NULL
    };

    bt_le_adv_stop();
    int err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("iBeacon adv start failed (err %d)", err);
    } else {
        // LOG_INF("iBeacon advertising distance: %u mm", distance_mm);
    }
}

/************************************************************************/
/* ULTRASONIC SENSOR FUNCTIONS                                          */
/************************************************************************/

/* Echo pin interrupt handler for sensor 1 */
static void echo1_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    int val = gpio_pin_get(echo_port, ULTRASONIC1_ECHO_PIN);
    
    if (val > 0) {
        /* Rising edge - start timing */
        echo1_start_time = k_cycle_get_32();
    } else {
        /* Falling edge - end timing and calculate duration */
        uint32_t end_time = k_cycle_get_32();
        echo1_pulse_duration = k_cyc_to_us_floor32(end_time - echo1_start_time);
        echo1_received = true;
    }
}

/* Calculate distance from pulse duration */
static uint16_t calculate_distance_mm(uint32_t pulse_duration_us)
{
    /* Distance = (Time × Speed of Sound) ÷ 2
     * Time is in microseconds, speed of sound is in meters per second
     * Convert to millimeters: multiply by 1000
     * Result: Distance (mm) = (Time (us) × Speed (m/s) × 1000) ÷ (2 × 1000000)
     * Simplified: Distance (mm) = (Time (us) × Speed (m/s)) ÷ 2000
     */
    if (pulse_duration_us > ULTRASONIC_TIMEOUT_USEC) {
        return 0; // Error or out of range
    }
    
    return (pulse_duration_us * SOUND_SPEED_MPS) / 2000;
}

/* Trigger ultrasonic measurement for sensor 1 */
static void trigger_sensor1(void)
{
    echo1_received = false;
    
    /* Send 10us pulse to trigger pin */
    gpio_pin_set(trig_port, ULTRASONIC1_TRIG_PIN, 1);
    k_busy_wait(10);
    gpio_pin_set(trig_port, ULTRASONIC1_TRIG_PIN, 0);
    
    /* Wait for echo with timeout */
    k_busy_wait(2); // Short delay to ensure trigger pulse is completed
    uint32_t start_wait = k_uptime_get_32();
    while (!echo1_received) {
        if (k_uptime_get_32() - start_wait > ULTRASONIC_TIMEOUT_USEC / 1000) {
            LOG_WRN("Sensor 1 timeout");
            break;
        }
        k_msleep(1);
    }
    
    if (echo1_received) {
        uint16_t new_distance = calculate_distance_mm(echo1_pulse_duration);
        
        k_mutex_lock(&distance_mutex, K_FOREVER);
        distance1_mm = new_distance;
        k_mutex_unlock(&distance_mutex);
        
        // LOG_INF("Sensor 1 distance: %u mm", distance1_mm);
    }
}

/* Initialize GPIO for pump control */
static int init_pump_gpio(void)
{
    if (!device_is_ready(trig_port)) { // Reusing the same GPIO port
        LOG_ERR("GPIO port not ready for pump control");
        return -ENODEV;
    }

    gpio_pin_configure(trig_port, PUMP_CONTROL_PIN, GPIO_OUTPUT);
    gpio_pin_set(trig_port, PUMP_CONTROL_PIN, 0); // Start with pump off
    
    LOG_INF("Pump control GPIO initialized on pin %d", PUMP_CONTROL_PIN);
    return 0;
}

/* Set pump state */
static void set_pump_state(bool active)
{
    k_mutex_lock(&pump_mutex, K_FOREVER);
    pump_active = active;
    k_mutex_unlock(&pump_mutex);
    
    gpio_pin_set(trig_port, PUMP_CONTROL_PIN, active ? 1 : 0);
    LOG_INF("Pump %s", active ? "ACTIVATED" : "DEACTIVATED");
}
static int init_ultrasonic_gpio(void)
{
    if (!device_is_ready(trig_port) || !device_is_ready(echo_port)) {
        LOG_ERR("GPIO ports not ready");
        return -ENODEV;
    }

    gpio_pin_configure(trig_port, ULTRASONIC1_TRIG_PIN, GPIO_OUTPUT);
    gpio_pin_configure(echo_port, ULTRASONIC1_ECHO_PIN, GPIO_INPUT);
    
    gpio_pin_interrupt_configure(echo_port, ULTRASONIC1_ECHO_PIN, GPIO_INT_EDGE_BOTH);

    /* Configure interrupt for echo pin 1 */
    gpio_init_callback(&echo1_cb_data, echo1_handler, BIT(ULTRASONIC1_ECHO_PIN));
    gpio_add_callback(echo_port, &echo1_cb_data);
    
    return 0;
}

/************************************************************************/
/* WIFI CONFIGURATION                                                   */
/************************************************************************/

#define NET_EVENT_WIFI_MASK                                                                        \
	(NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT |                        \
	 NET_EVENT_WIFI_AP_ENABLE_RESULT | NET_EVENT_WIFI_AP_DISABLE_RESULT |                      \
	 NET_EVENT_WIFI_AP_STA_CONNECTED | NET_EVENT_WIFI_AP_STA_DISCONNECTED)

/* Network management */
static struct net_mgmt_event_callback wifi_cb;
static bool wifi_connected_flag = false;
static struct net_if *sta_iface;
static struct wifi_connect_req_params sta_config;

static void wifi_event_handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event,
			       struct net_if *iface)
{
	switch (mgmt_event) {
	case NET_EVENT_WIFI_CONNECT_RESULT: {
		LOG_INF("Connected to %s", WIFI_SSID);
        wifi_connected_flag = true;
		break;
	}
	case NET_EVENT_WIFI_DISCONNECT_RESULT: {
		LOG_INF("Disconnected from %s", WIFI_SSID);
        wifi_connected_flag = false;
		break;
	}
	default:
		break;
	}
}

static int connect_to_wifi(void)
{
	if (!sta_iface) {
		LOG_INF("STA: interface not initialized");
		return -EIO;
	}

	sta_config.ssid = (const uint8_t *)WIFI_SSID;
	sta_config.ssid_length = strlen(WIFI_SSID);
	sta_config.psk = (const uint8_t *)WIFI_PSK;
	sta_config.psk_length = strlen(WIFI_PSK);
	sta_config.security = WIFI_SECURITY_TYPE_PSK;
	sta_config.channel = WIFI_CHANNEL_ANY;
	sta_config.band = WIFI_FREQ_BAND_2_4_GHZ;

	LOG_INF("Connecting to SSID: %s\n", sta_config.ssid);

	int ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, sta_iface, &sta_config,
			   sizeof(struct wifi_connect_req_params));
	if (ret) {
		LOG_ERR("Unable to Connect to (%s)", WIFI_SSID);
	}

	return ret;
}

void wifi_thread(void) {
    LOG_INF("Disco Board Ultrasonic Node Starting...");

    // Connect to wifi
    net_mgmt_init_event_callback(&wifi_cb, wifi_event_handler, NET_EVENT_WIFI_MASK);
    net_mgmt_add_event_callback(&wifi_cb);

	/* Get STA interface in AP-STA mode. */
	sta_iface = net_if_get_wifi_sta();

	connect_to_wifi();

    /* Wait for wifi connection */
    while (!wifi_connected_flag) {
        k_msleep(100);
    }
    LOG_INF("Connected to wifi");
}

K_THREAD_DEFINE(wifi_thread_id, 956, wifi_thread, NULL, NULL, NULL, 7, 0, 0);

/************************************************************************/
/* MQTT CONFIGURATION                                                   */
/************************************************************************/

/* Parse pump command from MQTT payload */
static bool parse_pump_command_from_payload(const uint8_t *payload, uint32_t len)
{
    char command_str[16];
    
    if (len >= sizeof(command_str)) {
        LOG_ERR("Payload too long");
        return false;
    }
    
    memcpy(command_str, payload, len);
    command_str[len] = '\0';
    
    // Convert to lowercase for comparison
    for (int i = 0; command_str[i]; i++) {
        command_str[i] = tolower(command_str[i]);
    }
    
    if (strcmp(command_str, "on") == 0 || strcmp(command_str, "1") == 0 || 
        strcmp(command_str, "true") == 0 || strcmp(command_str, "activate") == 0) {
        return true;
    } else if (strcmp(command_str, "off") == 0 || strcmp(command_str, "0") == 0 || 
               strcmp(command_str, "false") == 0 || strcmp(command_str, "deactivate") == 0) {
        return false;
    }
    
    LOG_WRN("Unknown pump command: %s", command_str);
    return false;
}

/* Process pump MQTT commands */
static void process_pump_mqtt_payload(struct mqtt_client *client, 
                                    const struct mqtt_publish_param *pub_param)
{
    if (!client || !pub_param) {
        return;
    }
    
    uint32_t payload_len = pub_param->message.payload.len;
    uint32_t topic_len = pub_param->message.topic.topic.size;
    
    if (payload_len == 0 || topic_len == 0) {
        return;
    }
    
    // Create buffers for topic and payload
    uint8_t topic_buffer[64];
    uint8_t payload_buffer[32];
    
    // Read topic
    uint32_t topic_bytes_to_read = topic_len < (sizeof(topic_buffer) - 1) ? 
                                   topic_len : (sizeof(topic_buffer) - 1);
    memcpy(topic_buffer, pub_param->message.topic.topic.utf8, topic_bytes_to_read);
    topic_buffer[topic_bytes_to_read] = '\0';
    
    // Read payload
    uint32_t payload_bytes_to_read = payload_len < (sizeof(payload_buffer) - 1) ? 
                                     payload_len : (sizeof(payload_buffer) - 1);
    int bytes_read = mqtt_read_publish_payload(client, payload_buffer, payload_bytes_to_read);
    
    if (bytes_read < 0) {
        LOG_ERR("Failed to read pump payload: %d", bytes_read);
        return;
    }
    
    payload_buffer[bytes_read] = '\0';
    
    // Check if this is the pump control topic
    if (strcmp((char*)topic_buffer, MQTT_TOPIC_PUMP_CONTROL) == 0) {
        bool pump_command = parse_pump_command_from_payload(payload_buffer, bytes_read);
        set_pump_state(pump_command);
        LOG_INF("Received pump command: %s -> %s", 
                payload_buffer, pump_command ? "ON" : "OFF");
    }
}

static void mqtt_evt_handler(struct mqtt_client *const client, const struct mqtt_evt *evt)
{
    switch (evt->type) {
    case MQTT_EVT_CONNACK:
        if (evt->result != 0) {
            LOG_ERR("MQTT connect failed %d", evt->result);
            break;
        }
        connected = true;
        LOG_INF("MQTT client connected!");
        break;

    case MQTT_EVT_DISCONNECT:
        LOG_INF("MQTT client disconnected %d", evt->result);
        connected = false;
        break;

    case MQTT_EVT_PUBLISH: {
        if (evt->result != 0) {
            LOG_ERR("MQTT PUBLISH error: %d", evt->result);
            break;
        }
        
        if (evt->param.publish.message.payload.len == 0) {
            break;
        }
        
        // Process pump command
        process_pump_mqtt_payload(client, &evt->param.publish);
        
        // Acknowledge if QoS 1
        if (evt->param.publish.message.topic.qos == MQTT_QOS_1_AT_LEAST_ONCE) {
            struct mqtt_puback_param puback = {
                .message_id = evt->param.publish.message_id
            };
            mqtt_publish_qos1_ack(client, &puback);
        }
        
        break;
    }

    case MQTT_EVT_PUBACK:
        if (evt->result != 0) {
            LOG_ERR("MQTT PUBACK error %d", evt->result);
            break;
        }
        LOG_DBG("PUBACK packet id: %u", evt->param.puback.message_id);
        break;

    case MQTT_EVT_SUBACK:
        if (evt->result != 0) {
            LOG_ERR("MQTT SUBACK error %d", evt->result);
            break;
        }
        LOG_INF("Successfully subscribed! SUBACK packet id: %u", evt->param.suback.message_id);
        break;

    case MQTT_EVT_UNSUBACK:
        LOG_INF("MQTT UNSUBACK packet id: %u", evt->param.unsuback.message_id);
        break;

    case MQTT_EVT_PINGRESP:
        LOG_DBG("MQTT PINGRESP received");
        break;

    default:
        LOG_DBG("Unhandled MQTT event type: %d", evt->type);
        break;
    }
}

static void broker_init(void)
{
    struct sockaddr_in *broker4 = (struct sockaddr_in *)&broker;

    broker4->sin_family = AF_INET;
    broker4->sin_port = htons(MQTT_BROKER_PORT);
    zsock_inet_pton(AF_INET, MQTT_BROKER_ADDR, &broker4->sin_addr);
}

static void client_init(struct mqtt_client *client)
{
    mqtt_client_init(client);

    broker_init();

    /* MQTT client configuration */
    client->broker = &broker;
    client->evt_cb = mqtt_evt_handler;
    client->client_id.utf8 = (uint8_t *)MQTT_CLIENT_ID;
    client->client_id.size = strlen(MQTT_CLIENT_ID);
    client->password = NULL;
    client->user_name = NULL;
    client->protocol_version = MQTT_VERSION_3_1_1;

    /* MQTT buffers configuration */
    client->rx_buf = rx_buffer;
    client->rx_buf_size = sizeof(rx_buffer);
    client->tx_buf = tx_buffer;
    client->tx_buf_size = sizeof(tx_buffer);

    /* MQTT transport configuration */
    client->transport.type = MQTT_TRANSPORT_NON_SECURE;
}

/* Subscribe to pump control topic */
static int subscribe_to_pump_topic(struct mqtt_client *client)
{
    struct mqtt_topic subs_topic;
    struct mqtt_subscription_list subs_list;

    subs_topic.topic.utf8 = (uint8_t *)MQTT_TOPIC_PUMP_CONTROL;
    subs_topic.topic.size = strlen(MQTT_TOPIC_PUMP_CONTROL);
    subs_topic.qos = MQTT_QOS_0_AT_MOST_ONCE;

    subs_list.list = &subs_topic;
    subs_list.list_count = 1U;
    subs_list.message_id = sys_rand32_get();

    int ret = mqtt_subscribe(client, &subs_list);
    if (ret == 0) {
        LOG_INF("Subscribed to pump control topic: %s", MQTT_TOPIC_PUMP_CONTROL);
    }
    
    return ret;
}

static void poll_mqtt_client(void)
{
    int ret;

    if (zsock_poll(fds, nfds, 1) < 0) {  // Very short timeout for non-blocking
        return;
    }

    if ((fds[0].revents & ZSOCK_POLLIN) == ZSOCK_POLLIN) {
        ret = mqtt_input(&client_ctx);
        if (ret != 0) {
            if (ret == -EAGAIN || ret == -EWOULDBLOCK) {
                LOG_DBG("mqtt_input: no more data available (%d)", ret);
            } else if (ret == -ENOTCONN) {
                LOG_ERR("mqtt_input: connection lost (%d)", ret);
                connected = false;
            } else if (ret == -EBUSY) {
                LOG_DBG("mqtt_input: busy, retrying later (%d)", ret);
            } else {
                LOG_ERR("mqtt_input error: %d", ret);
            }
            return;
        }
    }

    if ((fds[0].revents & (ZSOCK_POLLERR | ZSOCK_POLLHUP))) {
        LOG_ERR("MQTT socket error/hangup");
        connected = false;
    }
}

/************************************************************************/
/* ULTRASONIC SENSOR THREAD                                             */
/************************************************************************/

/* Ultrasonic sensor reading thread */
void ultrasonic_thread(void)
{
    int ret;
    uint16_t local_distance;
    
    LOG_INF("Starting ultrasonic sensor thread");
    
    /* Initialize ultrasonic GPIO */
    ret = init_ultrasonic_gpio();
    if (ret < 0) {
        LOG_ERR("Failed to initialize ultrasonic GPIO: %d", ret);
        return;
    }
    
    /* Initialize pump control GPIO */
    ret = init_pump_gpio();
    if (ret < 0) {
        LOG_ERR("Failed to initialize pump GPIO: %d", ret);
        return;
    }
    
    /* Initialize Bluetooth */
    ret = bt_enable(NULL);
    if (ret) {
        LOG_ERR("Bluetooth init failed (err %d)", ret);
        return;
    }
    LOG_INF("Bluetooth initialized");
    
    LOG_INF("Ultrasonic sensor ready. Starting measurements...");
    
    /* Main ultrasonic reading loop */
    while (1) {
        /* Trigger sensor measurement */
        trigger_sensor1();
        
        /* Get current distance reading */
        k_mutex_lock(&distance_mutex, K_FOREVER);
        local_distance = distance1_mm;
        k_mutex_unlock(&distance_mutex);
        
        /* Advertise distance via BLE iBeacon */
        advertise_distance(local_distance);
        
        k_msleep(MEASUREMENT_INTERVAL_MS);
    }
}

/* Create ultrasonic sensor thread */
K_THREAD_DEFINE(ultrasonic_thread_id, 1024, ultrasonic_thread, NULL, NULL, NULL, 6, 0, 0);

/************************************************************************/
/* MQTT THREAD                                                          */
/************************************************************************/

void mqtt_thread(void) {
    bool mqtt_initialized = false;
    
    /* Wait for wifi connection */
    while (!wifi_connected_flag) {
        k_msleep(100);
    }
    k_msleep(100);
    
    /* Initialize MQTT */
    LOG_INF("Initializing MQTT client");
    client_init(&client_ctx);

    /* Connect to MQTT broker */
    LOG_INF("Connecting to MQTT broker at %s:%d", MQTT_BROKER_ADDR, MQTT_BROKER_PORT);
    int ret = mqtt_connect(&client_ctx);
    if (ret != 0) {
        LOG_ERR("mqtt_connect failed: %d", ret);
        return;
    }

    /* Prepare for polling */
    fds[0].fd = client_ctx.transport.tcp.sock;
    fds[0].events = ZSOCK_POLLIN;
    nfds = 1;

    /* Wait for MQTT connection */
    while (!connected) {
        poll_mqtt_client();
        k_msleep(10);
    }
    mqtt_initialized = true;
    
    LOG_INF("MQTT setup complete. Starting main loop...");
    LOG_INF("Subscribed to pump control topic: %s", MQTT_TOPIC_PUMP_CONTROL);
    
    /* Subscribe to pump control topic */
    ret = subscribe_to_pump_topic(&client_ctx);
    if (ret != 0) {
        LOG_ERR("Failed to subscribe to pump topic: %d", ret);
    }

    /* Main MQTT loop */
    while (1) {
        // Handle MQTT
        if (mqtt_initialized) {
            poll_mqtt_client();
            if (connected) {
                mqtt_live(&client_ctx);  // Keep connection alive
            }
        }
        
        k_msleep(10);  // Short sleep to balance responsiveness
    }
}

K_THREAD_DEFINE(mqtt_thread_id, 1024*3, mqtt_thread, NULL, NULL, NULL, 7, 0, 0);

/************************************************************************/
/* MAIN FUNCTION                                                        */
/************************************************************************/

int main(void) {
    LOG_INF("Starting Disco Board Ultrasonic Node Application");
    
    /* Initialize distance and pump mutexes */
    k_mutex_init(&distance_mutex);
    k_mutex_init(&pump_mutex);
    
    return 0;
}