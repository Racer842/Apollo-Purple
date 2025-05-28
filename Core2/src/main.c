#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/display.h>
#include <zephyr/random/random.h>
#include <lvgl.h>
#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(core2_view, LOG_LEVEL_INF);

#define WIFI_SSID "Jake_iphone"
#define WIFI_PSK  "bbbooooo"

#define MQTT_BROKER_ADDR "172.20.10.11"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID "m5core2_display"
#define MQTT_TOPIC_SUB "m5core2/temp"

static struct mqtt_client client_ctx;
static uint8_t rx_buffer[128], tx_buffer[128];
static struct sockaddr_storage broker;
static struct zsock_pollfd fds[1];
static int nfds;
static bool connected = false;

static lv_obj_t *quads[4];
static lv_obj_t *quad_labels[4];
static float temps[4] = {29.3, 32.4, 29.3, 3.7}; // Initial values
static bool display_needs_update = false;

// Global display device
static const struct device *display_dev;

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

K_THREAD_DEFINE(wifi_thread_id, 956, wifi_thread, NULL, NULL, NULL, 7, 0, 0);

/************************************************************************/

// --- Commented out JSON parsing block for later use --- //

#include <zephyr/data/json.h>

struct Thingy {
	float temp;
};
struct Payload {
	struct Thingy thingy52[4];
};
static const struct json_obj_descr thingy_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct Thingy, temp, JSON_TOK_NUMBER)
};
static const struct json_obj_descr payload_descr[] = {
	JSON_OBJ_DESCR_OBJ_ARRAY(struct Payload, thingy52, 4, thingy_descr)
};
static int parse_temp_json(const char *buf) {
	struct Payload data;
	int ret = json_obj_parse_buf(buf, payload_descr, ARRAY_SIZE(payload_descr), &data);
	if (ret < 0) {
		LOG_ERR("JSON parse failed: %d", ret);
		return ret;
	}
	for (int i = 0; i < 4; i++) {
		temps[i] = data.thingy52[i].temp;
	}
	return 0;
}


static void process_mqtt_payload(struct mqtt_client *client, const struct mqtt_publish_param *pub_param)
{
    if (!client || !pub_param) {
        return;
    }
    
    uint32_t payload_len = pub_param->message.payload.len;
    
    if (payload_len == 0) {
        return;
    }
    
    // Create buffer for payload
    uint8_t payload_buffer[256];
    uint32_t buffer_size = sizeof(payload_buffer) - 1;
    uint32_t bytes_to_read = payload_len < buffer_size ? payload_len : buffer_size;
    
    // Use mqtt_read_publish_payload to safely read the payload
    int bytes_read = mqtt_read_publish_payload(client, payload_buffer, bytes_to_read);
    
    if (bytes_read < 0) {
        LOG_ERR("Failed to read payload: %d", bytes_read);
        return;
    }
    
    // Null terminate the message
    payload_buffer[bytes_read] = '\0';
    LOG_INF("Received: %s", payload_buffer);
    
    // Set flag to update display
    display_needs_update = true;
    
    // Uncomment below to enable JSON parsing
    
    if (parse_temp_json((const char*)payload_buffer) == 0) {
        display_needs_update = true;
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
        
        // Process the received message
        process_mqtt_payload(client, &evt->param.publish);
        
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

// Global persistent buffers for temperature labels
static char temp_label_buffers[4][16];

static void update_display(void) {
	// Only update if LVGL objects are created
	if (!quads[0] || !quad_labels[0]) {
		LOG_WRN("LVGL objects not ready for display update");
		return;
	}

	/* Determine the ranks of each temp */
	int rank[4] = {0};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			if (temps[i] > temps[j]) rank[i]++;
		}
	}

	/* Define relative colors */
	lv_color_t color_map[4] = {
		lv_color_make(255, 0, 0),     // coldest (blue)
		lv_color_make(0, 0, 255),     // mid-cold (cyan)
		lv_color_make(0, 255, 255),   // mid-warm (yellow)
		lv_color_hex(0x00FF00)        // hottest (red)
	};

	/* Apply color + label */
	for (int i = 0; i < 4; i++) {
        lv_color_t col = color_map[rank[i]];
        lv_obj_set_style_bg_color(quads[i], col, 0);

        // Use global persistent buffers and ensure proper formatting
        snprintf(temp_label_buffers[i], sizeof(temp_label_buffers[i]), "%d°C", (int)temps[i]);
        lv_label_set_text(quad_labels[i], temp_label_buffers[i]);
        
        LOG_DBG("Updated quad %d: temp=%d, rank=%d, label=%s", i, (int)temps[i], rank[i], temp_label_buffers[i]);
    }
}

// Combined MQTT + LVGL thread
void mqtt_lvgl_thread(void) {
    bool mqtt_initialized = false;
    bool lvgl_initialized = false;
    
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

    /* Subscribe to temperature topic */
    LOG_INF("Subscribing to topic: %s", MQTT_TOPIC_SUB);
    ret = subscribe_to_topic(&client_ctx, MQTT_TOPIC_SUB);
    if (ret != 0) {
        LOG_ERR("Failed to subscribe: %d", ret);
    } else {
        LOG_INF("Subscription request sent successfully");
    }

    /* Initialize LVGL */
    LOG_INF("Initializing LVGL");
    
    // Verify display device is available
    if (!display_dev || !device_is_ready(display_dev)) {
        LOG_ERR("Display device not ready");
        return;
    }

    lv_init();
    LOG_INF("LVGL initialized successfully");

    lv_obj_t *scr = lv_scr_act();
    lv_coord_t sw = lv_obj_get_width(scr);
    lv_coord_t sh = lv_obj_get_height(scr);

    // Initialize all quad objects to NULL first
    for (int i = 0; i < 4; i++) {
        quads[i] = NULL;
        quad_labels[i] = NULL;
    }

    for (int i = 0; i < 4; i++) {
        quads[i] = lv_obj_create(scr);
        lv_obj_set_size(quads[i], sw / 2, sh / 2);
        lv_obj_clear_flag(quads[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_border_width(quads[i], 1, 0);
        lv_obj_set_style_radius(quads[i], 0, 0);
        lv_obj_set_style_border_color(quads[i], lv_color_black(), 0);

        // Correct position based on index
        lv_obj_set_pos(quads[i],
                    (i % 2) * (sw / 2),
                    (i / 2) * (sh / 2));

        quad_labels[i] = lv_label_create(quads[i]);
        lv_obj_center(quad_labels[i]);
        lv_obj_set_style_text_color(quad_labels[i], lv_color_white(), 0);
        lv_obj_set_style_text_font(quad_labels[i], &lv_font_montserrat_28, 0);
        
        // Initialize with placeholder text first
        lv_label_set_text(quad_labels[i], "Loading...");
        
        LOG_INF("Created quad %d at position (%d, %d)", i, (i % 2) * (sw / 2), (i / 2) * (sh / 2));
    }

    // Small delay to ensure LVGL objects are fully created
    k_msleep(50);
    
    LOG_INF("Initial temperatures: %d, %d, %d, %d", 
            (int)temps[0], (int)temps[1], (int)temps[2], (int)temps[3]);
    
    update_display(); // draw initial temps
    lvgl_initialized = true;
    
    LOG_INF("MQTT and LVGL setup complete. Starting main loop...");
    LOG_INF("Listening for temperature messages on topic: %s", MQTT_TOPIC_SUB);

    /* Main combined loop */
    while (1) {
        // Handle MQTT
        if (mqtt_initialized) {
            poll_mqtt_client();
            if (connected) {
                mqtt_live(&client_ctx);  // Keep connection alive
            }
        }
        
        // Handle LVGL
        if (lvgl_initialized) {
            lv_timer_handler();
            
            // Update display if new data received
            if (display_needs_update) {
                update_display();
                display_needs_update = false;
            }
        }
        
        k_msleep(10);  // Short sleep to balance responsiveness
    }
}

int main(void) {
	LOG_INF("Starting M5 Core2 application");
	
	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(display_dev)) {
		LOG_ERR("Display not ready");
		return -1;
	}
	display_blanking_off(display_dev);
	
	LOG_INF("Display initialized, starting threads");
	return 0;
}

K_THREAD_DEFINE(mqtt_lvgl_thread_id, 1024*3, mqtt_lvgl_thread, NULL, NULL, NULL, 7, 0, 0);