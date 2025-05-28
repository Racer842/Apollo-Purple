#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/random/random.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(disco_servo, LOG_LEVEL_INF);

#define WIFI_SSID "Jake_iphone"
#define WIFI_PSK  "bbbooooo"

#define MQTT_BROKER_ADDR "172.20.10.11"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID "disco_servo_control"

/* MQTT Topics for Servo Control */
#define MQTT_TOPIC_PAN_ANGLE   "servo/pan/angle"
#define MQTT_TOPIC_TILT_ANGLE  "servo/tilt/angle"

static struct mqtt_client client_ctx;
static uint8_t rx_buffer[128], tx_buffer[128];
static struct sockaddr_storage broker;
static struct zsock_pollfd fds[1];
static int nfds;
static bool connected = false;

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
    LOG_INF("Disco Board Servo Control Starting...");

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

/* Parse angle from MQTT payload */
static int parse_angle_from_payload(const uint8_t *payload, uint32_t len)
{
    char angle_str[16];
    int angle;
    
    if (len >= sizeof(angle_str)) {
        LOG_ERR("Payload too long");
        return -1;
    }
    
    memcpy(angle_str, payload, len);
    angle_str[len] = '\0';
    
    angle = atoi(angle_str);
    
    if (angle < 0 || angle > 180) {
        LOG_ERR("Invalid angle: %d (must be 0-180)", angle);
        return -1;
    }
    
    return angle;
}

/* Process servo MQTT commands */
static void process_servo_mqtt_payload(struct mqtt_client *client, 
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
        LOG_ERR("Failed to read servo payload: %d", bytes_read);
        return;
    }
    
    payload_buffer[bytes_read] = '\0';
    
    // Parse angle from payload
    int angle = parse_angle_from_payload(payload_buffer, bytes_read);
    if (angle < 0) {
        return;
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
        
        // Process servo command
        process_servo_mqtt_payload(client, &evt->param.publish);
        
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

static int subscribe_to_topic(struct mqtt_client *client, const char *topic)
{
    struct mqtt_topic subs_topic;
    struct mqtt_subscription_list subs_list;

    subs_topic.topic.utf8 = (uint8_t *)topic;
    subs_topic.topic.size = strlen(topic);
    subs_topic.qos = MQTT_QOS_0_AT_MOST_ONCE;

    subs_list.list = &subs_topic;
    subs_list.list_count = 1U;
    subs_list.message_id = sys_rand32_get();

    return mqtt_subscribe(client, &subs_list);
}

/* Subscribe to servo control topics */
static int subscribe_to_servo_topics(struct mqtt_client *client)
{
    int ret;
    
    /* Subscribe to pan angle topic */
    ret = subscribe_to_topic(client, MQTT_TOPIC_PAN_ANGLE);
    if (ret != 0) {
        LOG_ERR("Failed to subscribe to pan topic: %d", ret);
        return ret;
    }
    
    /* Subscribe to tilt angle topic */
    ret = subscribe_to_topic(client, MQTT_TOPIC_TILT_ANGLE);
    if (ret != 0) {
        LOG_ERR("Failed to subscribe to tilt topic: %d", ret);
        return ret;
    }
    
    LOG_INF("Subscribed to servo topics: %s, %s", 
            MQTT_TOPIC_PAN_ANGLE, MQTT_TOPIC_TILT_ANGLE);
    
    return 0;
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
/* SERVO CONTROL THREAD                                                 */
/************************************************************************/

/* Servo control thread */
void servo_control_thread(void)
{
    int ret;
    int local_pan_angle, local_tilt_angle;
    bool angles_changed;
    
    LOG_INF("Starting servo control thread");
    
    /* Wait for WiFi connection */
    while (!wifi_connected_flag) {
        k_msleep(100);
    }
    
    /* Initialize servo PWM */
    ret = init_servo_pwm();
    if (ret < 0) {
        LOG_ERR("Failed to initialize servo PWM: %d", ret);
        return;
    }
    
    /* Wait for MQTT connection */
    while (!connected) {
        k_msleep(100);
    }
    
    /* Subscribe to servo topics */
    ret = subscribe_to_servo_topics(&client_ctx);
    if (ret != 0) {
        LOG_ERR("Failed to subscribe to servo topics");
        return;
    }
    
    LOG_INF("Servo control ready. Listening for angle commands...");
    
    /* Main servo control loop */
    while (1) {
        /* Check for angle updates */
        k_mutex_lock(&servo_angle_mutex, K_FOREVER);
        angles_changed = servo_angles_updated;
        if (angles_changed) {
            local_pan_angle = current_pan_angle;
            local_tilt_angle = current_tilt_angle;
            servo_angles_updated = false;
        }
        k_mutex_unlock(&servo_angle_mutex);
        
        /* Update servo positions if angles changed */
        if (angles_changed) {
            ret = set_servo_position(&pwm_servo_pan, local_pan_angle);
            if (ret < 0) {
                LOG_ERR("Failed to set pan servo position: %d", ret);
            }
            
            ret = set_servo_position(&pwm_servo_tilt, local_tilt_angle);
            if (ret < 0) {
                LOG_ERR("Failed to set tilt servo position: %d", ret);
            }
            
            LOG_INF("Servos updated - Pan: %d°, Tilt: %d°", 
                    local_pan_angle, local_tilt_angle);
        }
        
        k_msleep(50);  // 50ms update rate for servo control
    }
}

/* Create servo control thread */
K_THREAD_DEFINE(servo_thread_id, 1024, servo_control_thread, NULL, NULL, NULL, 6, 0, 0);

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
    LOG_INF("Listening for servo commands on topics: %s, %s", 
            MQTT_TOPIC_PAN_ANGLE, MQTT_TOPIC_TILT_ANGLE);

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
	LOG_INF("Starting Disco Board Servo Control Application");
	return 0;
}