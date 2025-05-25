#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <string.h>
#include <stdio.h>

LOG_MODULE_REGISTER(mqtt_client, LOG_LEVEL_DBG);

/* WiFi configuration - Update these with your network details */
#define WIFI_SSID "Jake_iphone"
#define WIFI_PSK "bbbooooo"

/* MQTT configuration - Update with your Mac's IP address */
#define MQTT_BROKER_ADDR "172.20.10.11"  // Replace with your Mac's IP
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID "m5core2_client"
#define MQTT_TOPIC_PUB "m5core2/status"
#define MQTT_TOPIC_SUB "m5core2/command"

/* Network management */
static struct net_mgmt_event_callback wifi_cb;

/* MQTT client configuration */
static uint8_t rx_buffer[128];
static uint8_t tx_buffer[128];
static struct mqtt_client client_ctx;
static struct sockaddr_storage broker;
static struct zsock_pollfd fds[1];
static int nfds;
static bool connected = false;

/* Message counter for demo */
static int message_count = 0;

static bool wifi_connected_flag = false;
static struct net_if *wifi_iface = NULL;

/************************************************************************/
#define NET_EVENT_WIFI_MASK                                                                        \
	(NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT |                        \
	 NET_EVENT_WIFI_AP_ENABLE_RESULT | NET_EVENT_WIFI_AP_DISABLE_RESULT |                      \
	 NET_EVENT_WIFI_AP_STA_CONNECTED | NET_EVENT_WIFI_AP_STA_DISCONNECTED)

/* STA Mode Configuration */
#define WIFI_SSID "Jake_iphone"
#define WIFI_PSK "bbbooooo"

static struct net_if *ap_iface;
static struct net_if *sta_iface;

static struct wifi_connect_req_params ap_config;
static struct wifi_connect_req_params sta_config;

static struct net_mgmt_event_callback cb;

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
		LOG_INF("STA: interface no initialized");
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
/************************************************************************/

/* Simplified and safer function to process received MQTT messages */
static void process_received_message(const struct mqtt_publish_param *pub_param)
{
    static char topic_str[64];
    static char message_str[128];
    
    /* Safety check for null parameter */
    if (!pub_param) {
        LOG_ERR("Null publish parameter");
        return;
    }
    
    /* Clear buffers */
    memset(topic_str, 0, sizeof(topic_str));
    memset(message_str, 0, sizeof(message_str));
    
    /* Extract topic with safety checks */
    if (pub_param->message.topic.topic.utf8 != NULL && 
        pub_param->message.topic.topic.size > 0 && 
        pub_param->message.topic.topic.size < sizeof(topic_str)) {
        
        memcpy(topic_str, pub_param->message.topic.topic.utf8, 
               pub_param->message.topic.topic.size);
        topic_str[pub_param->message.topic.topic.size] = '\0';
    } else {
        strncpy(topic_str, "Unknown", sizeof(topic_str) - 1);
    }
    
    /* Extract message payload with safety checks - FIXED */
    if (pub_param->message.payload.data != NULL && 
        pub_param->message.payload.len > 0 && 
        pub_param->message.payload.len < sizeof(message_str)) {
        
        /* Copy the payload data */
        memcpy(message_str, pub_param->message.payload.data, 
               pub_param->message.payload.len);
        message_str[pub_param->message.payload.len] = '\0';
        
        /* Debug: Print each byte to see what we're getting */
        LOG_DBG("Payload bytes:");
        for (int i = 0; i < pub_param->message.payload.len; i++) {
            LOG_DBG("  [%d]: 0x%02x ('%c')", i, 
                   ((uint8_t*)pub_param->message.payload.data)[i],
                   ((uint8_t*)pub_param->message.payload.data)[i]);
        }
    } else {
        strncpy(message_str, "Empty", sizeof(message_str) - 1);
        LOG_DBG("Payload data is NULL or invalid length: %d", pub_param->message.payload.len);
    }
    
    /* Simple logging */
    LOG_INF("MQTT RX - Topic: '%s', Message: '%s', Len: %d", 
            topic_str, message_str, pub_param->message.payload.len);
    
    /* Basic command processing */
    if (strstr(topic_str, "command") != NULL) {
        LOG_INF("Command received: %s", message_str);
    }
}

static void mqtt_evt_handler(struct mqtt_client *const client,
                            const struct mqtt_evt *evt)
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
        LOG_DBG("MQTT_EVT_PUBLISH received");
        
        if (evt->result != 0) {
            LOG_ERR("MQTT PUBLISH error: %d", evt->result);
            break;
        }
        
        /* Get publish parameter safely */
        const struct mqtt_publish_param *p = &evt->param.publish;
        
        /* Basic safety check */
        if (p && p->message.payload.len >= 0) {
            LOG_DBG("Payload length: %d", p->message.payload.len);
            process_received_message(p);
        } else {
            LOG_ERR("Invalid publish parameter");
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

static int publish_message(struct mqtt_client *client, const char *topic, const char *message)
{
    struct mqtt_publish_param param;
    
    param.message.topic.qos = MQTT_QOS_0_AT_MOST_ONCE;
    param.message.topic.topic.utf8 = (uint8_t *)topic;
    param.message.topic.topic.size = strlen(topic);
    param.message.payload.data = message;
    param.message.payload.len = strlen(message);
    param.message_id = sys_rand32_get();
    param.dup_flag = 0U;
    param.retain_flag = 0U;

    return mqtt_publish(client, &param);
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

static void poll_mqtt_client(void)
{
    int ret;

    if (zsock_poll(fds, nfds, 0) < 0) {
        return;
    }

    if ((fds[0].revents & ZSOCK_POLLIN) == ZSOCK_POLLIN) {
        ret = mqtt_input(&client_ctx);
        if (ret != 0) {
            /* Handle specific error codes */
            if (ret == -EAGAIN || ret == -EWOULDBLOCK) {
                /* This is normal - no more data available right now */
                LOG_DBG("mqtt_input: no more data available (%d)", ret);
            } else if (ret == -ENOTCONN) {
                LOG_ERR("mqtt_input: connection lost (%d)", ret);
                connected = false;
            } else {
                LOG_ERR("mqtt_input error: %d", ret);
            }
            return;
        }
    }

    if ((fds[0].revents & ZSOCK_POLLERR) == ZSOCK_POLLERR) {
        LOG_ERR("MQTT socket error");
        connected = false;
    }

    if ((fds[0].revents & ZSOCK_POLLHUP) == ZSOCK_POLLHUP) {
        LOG_ERR("MQTT socket hangup");
        connected = false;
    }
}

void wifi_thread (void) {

    LOG_INF("M5 CORE2 MQTT Client Starting...");

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
    LOG_INF("connected to wifi");
}

void mqtt_thread (void) {
    /* Wait for wifi connection */
    while (!wifi_connected_flag) {
        k_msleep(100);
    }
    k_msleep(100);
    int ret;
    char status_msg[64];

    /* Initialize MQTT client */
    client_init(&client_ctx);

    /* Connect to MQTT broker */
    LOG_INF("Connecting to MQTT broker at %s:%d", MQTT_BROKER_ADDR, MQTT_BROKER_PORT);
    ret = mqtt_connect(&client_ctx);
    if (ret != 0) {
        LOG_ERR("mqtt_connect failed: %d", ret);
        return ret;
    }

    /* Prepare for polling */
    fds[0].fd = client_ctx.transport.tcp.sock;
    fds[0].events = ZSOCK_POLLIN;
    nfds = 1;

    /* Wait for connection */
    while (!connected) {
        poll_mqtt_client();
        k_msleep(100);
    }

    /* Subscribe to command topic */
    LOG_INF("Subscribing to topic: %s", MQTT_TOPIC_SUB);
    ret = subscribe_to_topic(&client_ctx, MQTT_TOPIC_SUB);
    if (ret != 0) {
        LOG_ERR("Failed to subscribe: %d", ret);
    } else {
        LOG_INF("Subscription request sent successfully");
    }

    LOG_INF("MQTT setup complete. Starting main loop...");
    LOG_INF("Listening for messages on topic: %s", MQTT_TOPIC_SUB);

    /* Main loop */
    while (1) {
        poll_mqtt_client();

        /* Publish status message every 10 seconds */
        static uint64_t last_publish = 0;
        uint64_t now = k_uptime_get();
        
        if (now - last_publish >= 10000) { /* 10 seconds */
            if (connected) {
                snprintf(status_msg, sizeof(status_msg), 
                        "Hello from M5 CORE2! Count: %d, Uptime: %lld ms", 
                        message_count++, now);
                
                ret = publish_message(&client_ctx, MQTT_TOPIC_PUB, status_msg);
                if (ret != 0) {
                    LOG_ERR("Failed to publish: %d", ret);
                } else {
                    LOG_INF("Published: %s", status_msg);
                }
            }
            last_publish = now;
        }

        k_msleep(100);
    }
}

K_THREAD_DEFINE(mqtt_thread_id, 1024 * 2, mqtt_thread, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(wifi_thread_id, 1024 * 2, wifi_thread, NULL, NULL, NULL, 7, 0, 0);