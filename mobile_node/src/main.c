#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/sensor/ccs811.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/shell/shell.h>
#include <zephyr/version.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/bluetooth/bluetooth.h>

#include <zephyr/sys/printk.h>
#include <zephyr/shell/shell.h>
#include <zephyr/bluetooth/hci.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <zephyr/sys/slist.h>
#include <zephyr/data/json.h>

#define LED_RED_NODE    DT_ALIAS(led0)
#define LED_GREEN_NODE  DT_ALIAS(led1)
#define LED_BLUE_NODE   DT_ALIAS(led2)

LOG_MODULE_REGISTER(mobile_lib, LOG_LEVEL_INF);

const struct device *lis2dh = DEVICE_DT_GET_ONE(st_lis2dh);

static const struct gpio_dt_spec led_red = GPIO_DT_SPEC_GET(LED_RED_NODE, gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(LED_GREEN_NODE, gpios);
static const struct gpio_dt_spec led_blue = GPIO_DT_SPEC_GET(LED_BLUE_NODE, gpios);

/* Sensor reading threads */
#define STACK_SIZE 1024
#define PRIORITY  5

void set_rgb_color(bool r, bool g, bool b) {
    gpio_pin_set_dt(&led_red, r);
    gpio_pin_set_dt(&led_green, g);
    gpio_pin_set_dt(&led_blue, b);
}

static double x, y, z;

void lis2dh_thread(void) {
    struct sensor_value accel[3];
    while (1) {
        sensor_sample_fetch(lis2dh);
        sensor_channel_get(lis2dh, SENSOR_CHAN_ACCEL_XYZ, accel);

        x = sensor_value_to_double(&accel[0]);
        y = sensor_value_to_double(&accel[1]);
        z = sensor_value_to_double(&accel[2]);

        if (x < 0.6 && x > -0.6) {
            set_rgb_color(0, 1, 0);
        } else {
            set_rgb_color(1, 0, 0);
        }

        // printk("x: %.3f, y: %.3f, z: %.3f\n", x, y, z);

        k_msleep(500);
    }
}

K_THREAD_DEFINE(lis2dh_tid, STACK_SIZE, lis2dh_thread, NULL, NULL, NULL, 3, 0, 0);

/**********************************************************************************/

#define MAX_NODES 4
#define NODE_TIMEOUT_MS 5000  // 5 seconds timeout for node data

// Node tracking structure
typedef struct {
    uint8_t node_id;        // 1=A, 2=B, 3=C, 4=D
    int8_t rssi;
    uint8_t raw_data[8];    // Store the last 8 bytes of iBeacon data
    int64_t last_seen;
    bool valid;
} node_data_t;

static node_data_t detected_nodes[MAX_NODES];
static K_MUTEX_DEFINE(nodes_mutex);

// Target UUIDs for each node (first 12 bytes)
static const uint8_t target_uuid_a[12] = {
    0xAA, 0x01, 0x01, 0x01,  // Node A identifier
    0xFF, 0xFB, 0x48, 0xD2,
    0xB0, 0x60, 0xD0, 0xF5,
};
static const uint8_t target_uuid_b[12] = {
    0xBB, 0x02, 0x02, 0x02,  // Node B identifier
    0xFF, 0xFB, 0x48, 0xD2,
    0xB0, 0x60, 0xD0, 0xF5,
};
static const uint8_t target_uuid_c[12] = {
    0xCC, 0x03, 0x03, 0x03,  // Node C identifier
    0xFF, 0xFB, 0x48, 0xD2,
    0xB0, 0x60, 0xD0, 0xF5,
};
static const uint8_t target_uuid_d[12] = {
    0xDD, 0x04, 0x04, 0x04,  // Node D identifier
    0xFF, 0xFB, 0x48, 0xD2,
    0xB0, 0x60, 0xD0, 0xF5,
};

// New UUID template for re-advertising (with flag at 12th byte)
static const uint8_t relay_uuid_base[11] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB, 0x48, 0xD2,
    0xB0, 0x60, 0xD0
};

// Function to find the closest node
static uint8_t find_closest_node(void) {
    k_mutex_lock(&nodes_mutex, K_FOREVER);
    
    int8_t strongest_rssi = -127;  // Weakest possible RSSI
    uint8_t closest_node = 0;
    int64_t current_time = k_uptime_get();
    
    // Check each node and find the one with strongest RSSI
    for (int i = 0; i < MAX_NODES; i++) {
        if (detected_nodes[i].valid && 
            (current_time - detected_nodes[i].last_seen) < NODE_TIMEOUT_MS) {
            
            if (detected_nodes[i].rssi > strongest_rssi) {
                strongest_rssi = detected_nodes[i].rssi;
                closest_node = detected_nodes[i].node_id;
            }
        }
    }
    
    k_mutex_unlock(&nodes_mutex);
    return closest_node;
}

// Function to get node data by ID
static bool get_node_data(uint8_t node_id, node_data_t *data) {
    k_mutex_lock(&nodes_mutex, K_FOREVER);
    
    for (int i = 0; i < MAX_NODES; i++) {
        if (detected_nodes[i].node_id == node_id && detected_nodes[i].valid) {
            int64_t current_time = k_uptime_get();
            if ((current_time - detected_nodes[i].last_seen) < NODE_TIMEOUT_MS) {
                memcpy(data, &detected_nodes[i], sizeof(node_data_t));
                k_mutex_unlock(&nodes_mutex);
                return true;
            }
        }
    }
    
    k_mutex_unlock(&nodes_mutex);
    return false;
}

// Function to update node data
static void update_node_data(uint8_t node_id, int8_t rssi, const uint8_t *data) {
    k_mutex_lock(&nodes_mutex, K_FOREVER);
    
    // Find existing node or empty slot
    int slot = -1;
    for (int i = 0; i < MAX_NODES; i++) {
        if (detected_nodes[i].node_id == node_id) {
            slot = i;
            break;
        }
        if (!detected_nodes[i].valid && slot == -1) {
            slot = i;
        }
    }
    
    if (slot != -1) {
        detected_nodes[slot].node_id = node_id;
        detected_nodes[slot].rssi = rssi;
        detected_nodes[slot].last_seen = k_uptime_get();
        detected_nodes[slot].valid = true;
        memcpy(detected_nodes[slot].raw_data, data, 8);
    }
    
    k_mutex_unlock(&nodes_mutex);
}

static void device_found(const bt_addr_le_t *addr,
                        int8_t rssi, uint8_t type,
                        struct net_buf_simple *adp) {
    // Only process when device is level (accelerometer check)
    if (x < 0.6 && x > -0.6) {
        uint8_t len = adp->len;
        uint8_t *data = adp->data;

        while (len > 1) {
            uint8_t alen = data[0], atype = data[1];
            if (alen + 1 > len) break;

            // Check for iBeacon manufacturer data
            if (atype == BT_DATA_MANUFACTURER_DATA && alen >= 23 && 
                data[2] == 0x4C && data[3] == 0x00 && 
                data[4] == 0x02 && data[5] == 0x15) {

                // Check which node this UUID matches
                uint8_t node_id = 0;
                if (memcmp(&data[6], target_uuid_a, 12) == 0) {
                    node_id = 1; // Node A
                } else if (memcmp(&data[6], target_uuid_b, 12) == 0) {
                    node_id = 2; // Node B
                } else if (memcmp(&data[6], target_uuid_c, 12) == 0) {
                    node_id = 3; // Node C
                } else if (memcmp(&data[6], target_uuid_d, 12) == 0) {
                    node_id = 4; // Node D
                }

                if (node_id > 0) {
                    // Update node data with RSSI and payload
                    update_node_data(node_id, rssi, &data[21]); // Last 8 bytes
                    
                    // printk("Node %c detected: RSSI=%d\n", 'A' + node_id - 1, rssi);
                }
                break;
            }
            data += alen + 1;
            len  -= alen + 1;
        }
    }
}

/* Reusable function to update and advertise */
static void advertise_closest_node(void)
{
    uint8_t closest = find_closest_node();
    
    if (closest == 0) {
        // No valid nodes found
        return;
    }
    
    node_data_t node_data;
    if (!get_node_data(closest, &node_data)) {
        // Node data not available
        return;
    }
    
    uint8_t adv_data[30] = {
        0x02, 0x01, 0x06,                         // Flags
        0x1A, 0xFF,                               // Length, Manufacturer Specific
        0x4C, 0x00,                               // Apple Company ID
        0x02, 0x15                                // iBeacon type and length
    };

    // Copy the relay UUID base (11 bytes)
    memcpy(&adv_data[9], relay_uuid_base, 11);
    
    // Set the 12th byte (index 20) as the node flag
    adv_data[20] = closest;  // 1=A, 2=B, 3=C, 4=D
    // Copy the original node's data (last 8 bytes)
    memcpy(&adv_data[21], node_data.raw_data, 8);

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
        printk("Relay adv start failed (err %d)\n", err);
    } else {
        printk("Relaying Node %c data (RSSI: %d)\n", 'A' + closest - 1, node_data.rssi);
    }
}

static int observer_start(void) {
    struct bt_le_scan_param sp = {
        .type     = BT_LE_SCAN_TYPE_PASSIVE,
        .options  = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
        .interval = 0x0004,
        .window   = 0x0004,
    };
    int err = bt_le_scan_start(&sp, device_found);
    if (err) {
        printk("Scan failed (err %d)\n", err);
        return err;
    }

    printk("BLE scanning started\n");
    return 0;
}

// Thread to periodically advertise the closest node's data
void advertising_thread(void) {
    while (1) {
        // Only advertise when device is level
        if (x < 0.6 && x > -0.6) {
            advertise_closest_node();
        }
        
        k_msleep(1000); // Update every second
    }
}

K_THREAD_DEFINE(adv_tid, STACK_SIZE, advertising_thread, NULL, NULL, NULL, 4, 0, 0);

int main(void) {
    // Initialize GPIO for LEDs
    gpio_pin_configure_dt(&led_red, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led_blue, GPIO_OUTPUT_ACTIVE);

    set_rgb_color(1, 0, 0);
    
    // Initialize detected nodes array
    memset(detected_nodes, 0, sizeof(detected_nodes));
    
    LOG_INF("Mobile Node Initialised.");

    if (bt_enable(NULL)) {
        printk("BT init failed\n");
        return -1;
    }
    
    observer_start();

    return 0;
}