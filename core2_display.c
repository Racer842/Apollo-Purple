/*
 * M5 Core2 Zephyr Application
 * Wi-Fi Communication + Touch Display + MQTT Control
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/display.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/rand32.h>
#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(m5_core2_display, LOG_LEVEL_DBG);

/* Configuration */
#define WIFI_SSID "YourWiFiSSID"
#define WIFI_PSK "YourWiFiPassword"
#define MQTT_BROKER_IP "192.168.1.100"  // Your Mac IP
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID "M5Core2_Display"
#define MQTT_TOPIC_SENSOR "iot/sensors/data"
#define MQTT_TOPIC_CONTROL "iot/m5core2/control"
#define MQTT_TOPIC_STATUS "iot/m5core2/status"

/* Display configuration */
#define DISPLAY_WIDTH 320
#define DISPLAY_HEIGHT 240
#define FONT_HEIGHT 16
#define FONT_WIDTH 8

/* Button GPIO pins for M5 Core2 */
#define BUTTON_A_PIN 39
#define BUTTON_B_PIN 38
#define BUTTON_C_PIN 37

/* Network status */
static bool wifi_connected = false;
static bool mqtt_connected = false;

/* MQTT client and buffers */
static struct mqtt_client client;
static uint8_t rx_buffer[512];
static uint8_t tx_buffer[512];
static struct sockaddr_storage broker;

/* Display device */
static const struct device *display_dev;

/* Sensor data storage */
struct sensor_reading {
    char device_id[32];
    float temperature;
    float humidity;
    uint32_t timestamp;
    bool valid;
};

static struct sensor_reading latest_reading = {0};
static int total_messages = 0;
static uint32_t last_update = 0;

/* Button states */
static bool button_a_pressed = false;
static bool button_b_pressed = false;
static bool button_c_pressed = false;

/* Display buffer */
static char display_buffer[10][40]; // 10 lines, 40 chars each
static int display_line = 0;

/* Network management callback */
static struct net_mgmt_event_callback mgmt_cb;

/* Function prototypes */
static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
                                   uint32_t mgmt_event, struct net_if *iface);
static int wifi_connect(void);
static int mqtt_connect(void);
static void mqtt_evt_handler(struct mqtt_client *const client,
                            const struct mqtt_evt *evt);
static void update_display(void);
static void clear_display_buffer(void);
static void add_display_line(const char *text);
static void process_sensor_message(const char *payload);
static void publish_status_message(const char *status);

/* Wi-Fi Management Event Handler */
static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
                                   uint32_t mgmt_event, struct net_if *iface)
{
    switch (mgmt_event) {
    case NET_EVENT_WIFI_CONNECT_RESULT:
        LOG_INF("Wi-Fi connected");
        wifi_connected = true;
        add_display_line("WiFi: Connected");
        break;
    case NET_EVENT_WIFI_DISCONNECT_RESULT:
        LOG_INF("Wi-Fi disconnected");
        wifi_connected = false;
        mqtt_connected = false;
        add_display_line("WiFi: Disconnected");
        break;
    default:
        break;
    }
    update_display();
}

/* Wi-Fi Connection */
static int wifi_connect(void)
{
    struct net_if *iface = net_if_get_default();
    struct wifi_connect_req_params params = {0};

    params.ssid = WIFI_SSID;
    params.ssid_length = strlen(WIFI_SSID);
    params.psk = WIFI_PSK;
    params.psk_length = strlen(WIFI_PSK);
    params.channel = WIFI_CHANNEL_ANY;
    params.security = WIFI_SECURITY_TYPE_PSK;

    LOG_INF("Connecting to Wi-Fi: %s", WIFI_SSID);
    add_display_line("WiFi: Connecting...");
    update_display();
    
    return net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &params, sizeof(params));
}

/* MQTT Event Handler */
static void mqtt_evt_handler(struct mqtt_client *const client,
                            const struct mqtt_evt *evt)
{
    switch (evt->type) {
    case MQTT_EVT_CONNACK:
        if (evt->result == 0) {
            LOG_INF("MQTT connected");
            mqtt_connected = true;
            add_display_line("MQTT: Connected");
            
            /* Subscribe to sensor data topic */
            struct mqtt_subscription_list subscription_list = {
                .list = (struct mqtt_topic[]){
                    {
                        .topic = {
                            .utf8 = MQTT_TOPIC_SENSOR,
                            .size = strlen(MQTT_TOPIC_SENSOR)
                        },
                        .qos = MQTT_QOS_1_AT_LEAST_ONCE
                    }
                },
                .list_count = 1,
                .message_id = sys_rand32_get()
            };
            
            mqtt_subscribe(client, &subscription_list);
            publish_status_message("M5Core2 Online");
        } else {
            LOG_ERR("MQTT connection failed: %d", evt->result);
            add_display_line("MQTT: Failed");
        }
        update_display();
        break;
        
    case MQTT_EVT_DISCONNECT:
        LOG_INF("MQTT disconnected");
        mqtt_connected = false;
        add_display_line("MQTT: Disconnected");
        update_display();
        break;
        
    case MQTT_EVT_PUBLISH:
        if (evt->param.publish.message.payload.len > 0) {
            char payload[256];
            size_t len = MIN(evt->param.publish.message.payload.len, sizeof(payload) - 1);
            memcpy(payload, evt->param.publish.message.payload.data, len);
            payload[len] = '\0';
            
            LOG_INF("Received MQTT message: %s", payload);
            process_sensor_message(payload);
        }
        break;
        
    default:
        break;
    }
}

/* MQTT Connection Setup */
static int mqtt_connect(void)
{
    struct sockaddr_in *broker4 = (struct sockaddr_in *)&broker;
    int ret;

    mqtt_client_init(&client);

    broker4->sin_family = AF_INET;
    broker4->sin_port = htons(MQTT_BROKER_PORT);
    inet_pton(AF_INET, MQTT_BROKER_IP, &broker4->sin_addr);

    client.broker = &broker;
    client.evt_cb = mqtt_evt_handler;
    client.client_id.utf8 = MQTT_CLIENT_ID;
    client.client_id.size = strlen(MQTT_CLIENT_ID);
    client.password = NULL;
    client.user_name = NULL;
    client.protocol_version = MQTT_VERSION_3_1_1;
    client.rx_buf = rx_buffer;
    client.rx_buf_size = sizeof(rx_buffer);
    client.tx_buf = tx_buffer;
    client.tx_buf_size = sizeof(tx_buffer);
    client.transport.type = MQTT_TRANSPORT_NON_SECURE;

    ret = mqtt_connect(&client);
    if (ret) {
        LOG_ERR("MQTT connect failed: %d", ret);
        return ret;
    }

    LOG_INF("Connecting to MQTT broker at %s:%d", MQTT_BROKER_IP, MQTT_BROKER_PORT);
    return 0;
}

/* Display Buffer Management */
static void clear_display_buffer(void)
{
    memset(display_buffer, 0, sizeof(display_buffer));
    display_line = 0;
}

static void add_display_line(const char *text)
{
    if (display_line >= 10) {
        /* Scroll up */
        for (int i = 0; i < 9; i++) {
            strcpy(display_buffer[i], display_buffer[i + 1]);
        }
        display_line = 9;
    }
    
    strncpy(display_buffer[display_line], text, 39);
    display_buffer[display_line][39] = '\0';
    display_line++;
}

/* Update Display */
static void update_display(void)
{
    if (!display_dev) {
        return;
    }

    /* Clear display */
    display_blanking_off(display_dev);
    
    /* Create status header */
    char header[80];
    snprintf(header, sizeof(header), "M5Core2 IoT Monitor");
    
    char status_line[80];
    snprintf(status_line, sizeof(status_line), "WiFi:%s MQTT:%s Msgs:%d",
             wifi_connected ? "OK" : "NO",
             mqtt_connected ? "OK" : "NO",
             total_messages);
    
    char sensor_line[80];
    if (latest_reading.valid) {
        snprintf(sensor_line, sizeof(sensor_line), 
                "T:%.1fC H:%.1f%% Dev:%s",
                latest_reading.temperature,
                latest_reading.humidity,
                latest_reading.device_id);
    } else {
        strcpy(sensor_line, "No sensor data");
    }
    
    /* Display implementation would depend on your display driver */
    /* This is a placeholder showing the structure */
    LOG_INF("Display Update:");
    LOG_INF("Header: %s", header);
    LOG_INF("Status: %s", status_line);
    LOG_INF("Sensor: %s", sensor_line);
    
    for (int i = 0; i < display_line && i < 7; i++) {
        LOG_INF("Line %d: %s", i, display_buffer[i]);
    }
}

/* Process incoming sensor messages */
static void process_sensor_message(const char *payload)
{
    /* Simple JSON parsing - in production use a proper JSON parser */
    char *temp_str = strstr(payload, "\"temperature\":");
    char *humidity_str = strstr(payload, "\"humidity\":");
    char *device_str = strstr(payload, "\"deviceId\":\"");
    
    if (temp_str && humidity_str) {
        sscanf(temp_str, "\"temperature\":%f", &latest_reading.temperature);
        sscanf(humidity_str, "\"humidity\":%f", &latest_reading.humidity);
        
        if (device_str) {
            char *end_quote = strchr(device_str + 12, '"');
            if (end_quote) {
                size_t len = MIN(end_quote - (device_str + 12), sizeof(latest_reading.device_id) - 1);
                strncpy(latest_reading.device_id, device_str + 12, len);
                latest_reading.device_id[len] = '\0';
            }
        }
        
        latest_reading.timestamp = k_uptime_get_32() / 1000;
        latest_reading.valid = true;
        last_update = latest_reading.timestamp;
        total_messages++;
        
        char msg_line[80];
        snprintf(msg_line, sizeof(msg_line), "Rx: %.1fC %.1f%% (%s)",
                latest_reading.temperature, latest_reading.humidity,
                latest_reading.device_id);
        add_display_line(msg_line);
        update_display();
    }
}

/* Publish status message */
static void publish_status_message(const char *status)
{
    struct mqtt_publish_param param;
    char payload[128];
    
    if (!mqtt_connected) {
        return;
    }
    
    snprintf(payload, sizeof(payload),
             "{"
             "\"device\":\"M5Core2\","
             "\"status\":\"%s\","
             "\"timestamp\":%u,"
             "\"uptime\":%u"
             "}",
             status, (uint32_t)(k_uptime_get_32() / 1000), 
             (uint32_t)(k_uptime_get_32() / 1000));

    param.message.topic.qos = MQTT_QOS_1_AT_LEAST_ONCE;
    param.message.topic.topic.utf8 = MQTT_TOPIC_STATUS;
    param.message.topic.topic.size = strlen(MQTT_TOPIC_STATUS);
    param.message.payload.data = payload;
    param.message.payload.len = strlen(payload);
    param.message_id = sys_rand32_get();
    param.dup_flag = 0;
    param.retain_flag = 0;

    mqtt_publish(&client, &param);
}

/* Button handling */
static void handle_buttons(void)
{
    /* Button A: Refresh display */
    if (button_a_pressed) {
        button_a_pressed = false;
        add_display_line("Button A: Refresh");
        update_display();
        publish_status_message("Button A Pressed");
    }
    
    /* Button B: Send test message */
    if (button_b_pressed) {
        button_b_pressed = false;
        add_display_line("Button B: Test");
        publish_status_message("Test Message");
        update_display();
    }
    
    /* Button C: Clear display */
    if (button_c_pressed) {
        button_c_pressed = false;
        clear_display_buffer();
        add_display_line("Display Cleared");
        update_display();
    }
}

/* Main display thread */
static void display_thread(void)
{
    while (1) {
        handle_buttons();
        
        /* Check for stale data */
        uint32_t current_time = k_uptime_get_32() / 1000;
        if (latest_reading.valid && (current_time - last_update) > 30) {
            add_display_line("Warning: Stale data");
            update_display();
        }
        
        k_sleep(K_MSEC(100));
    }
}

/* MQTT maintenance thread */
static void mqtt_thread(void)
{
    while (1) {
        if (wifi_connected && mqtt_connected) {
            mqtt_live(&client);
        } else if (wifi_connected && !mqtt_connected) {
            LOG_INF("Attempting MQTT connection...");
            add_display_line("MQTT: Connecting...");
            update_display();
            mqtt_connect();
        }
        
        k_sleep(K_MSEC(1000));
    }
}

/* Status reporting thread */
static void status_thread(void)
{
    while (1) {
        if (mqtt_connected) {
            publish_status_message("Heartbeat");
        }
        k_sleep(K_MSEC(30000)); // Every 30 seconds
    }
}

/* Define threads */
K_THREAD_DEFINE(display_tid, 2048, display_thread, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(mqtt_tid, 2048, mqtt_thread, NULL, NULL, NULL, 6, 0, 0);
K_THREAD_DEFINE(status_tid, 1024, status_thread, NULL, NULL, NULL, 8, 0, 0);

int main(void)
{
    LOG_INF("M5 Core2 IoT Display Starting...");

    /* Initialize display */
    display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(display_dev)) {
        LOG_ERR("Display device not ready");
        display_dev = NULL;
    } else {
        LOG_INF("Display initialized");
        display_blanking_off(display_dev);
    }

    /* Initialize display buffer */
    clear_display_buffer();
    add_display_line("M5Core2 Starting...");
    add_display_line("Initializing...");
    update_display();

    /* Setup Wi-Fi management */
    net_mgmt_init_event_callback(&mgmt_cb, wifi_mgmt_event_handler,
                                NET_EVENT_WIFI_CONNECT_RESULT |
                                NET_EVENT_WIFI_DISCONNECT_RESULT);
    net_mgmt_add_event_callback(&mgmt_cb);

    /* Connect to Wi-Fi */
    wifi_connect();

    LOG_INF("M5Core2 system initialized");
    add_display_line("System Ready");
    update_display();

    return 0;
}