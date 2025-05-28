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

        printk("x: %.3f, y: %.3f, z: %.3f\n", x, y, z);

        k_msleep(500);
    }
}

K_THREAD_DEFINE(lis2dh_tid, STACK_SIZE, lis2dh_thread, NULL, NULL, NULL, 3, 0, 0);

int main(void) {

    gpio_pin_configure_dt(&led_red, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led_blue, GPIO_OUTPUT_ACTIVE);

    set_rgb_color(1, 0, 0);
    
    LOG_INF("Mobile Node Initialised.");

    return 0;
}

/**********************************************************************************/


 
 
#define MAX_NODES 4

struct beacon_json {
    char   *node_id[MAX_NODES];
    char   *rssi_vals[MAX_NODES];
    int     len_values;
    char   *pos_info[2];
    int     pos_info_len;
};

typedef struct beacon {
    char           id[8];            /* 'A'…'M' */
    uint16_t       major;         /* iBeacon major */
    uint16_t       minor;         /* iBeacon minor */
    char           mac[18];       /* "AA:BB:CC:DD:EE:FF" */
    double         x, y;          /* grid coords */
    bool           active;        /* set by CLI */
    sys_snode_t    node;          /* for Zephyr’s sys_slist */
} beacon_t;

double Ultra_vale;

static const char *ids[]     = {"4011-A","4011-B","4011-C","4011-D","4011-E","4011-F","4011-G","4011-H","4011-I","4011-J","4011-K","4011-L","4011-M"};
static const uint16_t majors[]  = {2753,32975,26679,41747,30679,6195,30525,57395,60345,12249,36748,27564,49247};
static const uint16_t minors[]  = {32998,20959,40363,38800,51963,18394,30544,28931,49995,30916,11457,27589,52925};
static const char    *mac_addrs[]= {
    "F5:75:FE:85:34:67","E5:73:87:06:1E:86","CA:99:9E:FD:98:B1","CB:1B:89:82:FF:FE",
    "D4:D2:A0:A4:5C:AC","C1:13:27:E9:B7:7C","F1:04:48:06:39:A0","CA:0C:E0:DB:CE:60",
    "D4:7F:D4:7C:20:13","F7:0B:21:F1:C8:E1","FD:E0:8D:FA:3E:4A","EE:32:F7:28:FA:AC",
    "F7:3B:46:A8:D7:2C"
};
static const double   node_x[]  = {0.0,1.5,3.0,3.0,3.0,1.5,0.0,0.0,4.5,6.0,6.0,6.0,4.5};
static const double   node_y[]  = {0.0,0.0,0.0,2.0,4.0,4.0,4.0,2.0,0.0,0.0,2.0,4.0,4.0};

/* -------------------------------------------------- */
/* 2) BLE iBeacon payload & advertising data         */
/* -------------------------------------------------- */
static uint8_t ibeacon_data[] = {
    /* Flags */              0x02,0x01,0x06,
    /* Len=0x1A,Type=0xFF */ 0x1A,0xFF,
    /* Apple Company */      0x4C,0x00,
    /* iBeacon marker */     0x02,0x15,
    /* 16-byte UUID */       0xFF,0xFF,0xFF,0xFF, 0xFF,0xFF,0xFF,0xFF,
                            0xFF,0xCC,0xCC,0xCC, 0x00,0x00,0x00,0x00,
    /* Major */              0x00,0x00,
    /* Minor */              0x00,0x00,
    /* Power @1m */          0xC8
};
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS,
                BT_LE_AD_GENERAL|BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, &ibeacon_data[5], 25)
};


struct node_state { 
    double dist;
    int8_t rssi;
    int64_t ts_ms;
    bool seen; };
static struct node_state nodes[MAX_NODES];

static double rssi_to_dist(int8_t rssi) {
    double power = ((-56 - rssi)/(10.0*2.0));  /* path-loss exponent */
    return pow(10.0, power);
}

static void device_found(const bt_addr_le_t *addr,
                        int8_t rssi, uint8_t type,
                        struct net_buf_simple *adp) {
    if (x < 0.6 && x > -0.6) {
        uint8_t len = adp->len;
        uint8_t *data = adp->data;

        while (len > 1) {
            uint8_t alen = data[0], atype = data[1];
            if (alen + 1 > len) break;

            if (atype == BT_DATA_MANUFACTURER_DATA && alen >= 23 && data[2]==0x4C && data[3]==0x00 && data[4]==0x02 
                && data[5] ==0x15) {

                
                /* Match your target UUID */
                static const uint8_t target_uuid_a[12] = {
                    0xAA, 0x01, 0x01, 0x01,  // First 4 bytes - node identifier (preset 1)
                    0xFF, 0xFB, 0x48, 0xD2,
                    0xB0, 0x60, 0xD0, 0xF5,
                };
                static const uint8_t target_uuid_b[12] = {
                    0xBB, 0x02, 0x02, 0x02,  // First 4 bytes - node identifier (preset 1)
                    0xFF, 0xFB, 0x48, 0xD2,
                    0xB0, 0x60, 0xD0, 0xF5,
                };
                static const uint8_t target_uuid_c[12] = {
                    0xCC, 0x03, 0x03, 0x03,  // First 4 bytes - node identifier (preset 1)
                    0xFF, 0xFB, 0x48, 0xD2,
                    0xB0, 0x60, 0xD0, 0xF5,
                };
                static const uint8_t target_uuid_d[12] = {
                    0xDD, 0x04, 0x04, 0x04,  // First 4 bytes - node identifier (preset 1)
                    0xFF, 0xFB, 0x48, 0xD2,
                    0xB0, 0x60, 0xD0, 0xF5,
                };

                bool uuid_match_a = true;
                bool uuid_match_b = true;
                bool uuid_match_c = true;
                bool uuid_match_d = true;
                for (int i = 0; i < 12; i++) {
                    if (data[6 + i] != target_uuid_a[i]) {
                        uuid_match_a = false;
                    } else if (data[6 + i] != target_uuid_b[i]) {
                        uuid_match_b = false;
                    } else if (data[6 + i] != target_uuid_c[i]) {
                        uuid_match_c = false;
                    } else if (data[6 + i] != target_uuid_d[i]) {
                        uuid_match_d = false;
                    }   
                }

                if (uuid_match_a || uuid_match_b || uuid_match_c || uuid_match_d) {
                    advertise_node(data);
                }
            
                break;
            }
            data += alen + 1;
            len  -= alen + 1;
        }
    }
}

/* -------------------------------------------------- */
/* 6) Multilateration thread & advertising update     */
/* -------------------------------------------------- */
/* ======================= Bluetooth iBeacon Section ======================= */

#define IBEACON_PREFIX_LEN 9
#define IBEACON_UUID_PREFIX_LEN 12
#define IBEACON_PAYLOAD_LEN 30
#define IBEACON_DATA_LEN 8

static const uint8_t beacon_uuid[12] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB, 0x48, 0xD2,
    0xB0, 0x60, 0xD0, 0xF5
};

/* Reusable function to update and advertise */
static void advertise_node(uint8_t *data)
{
    uint8_t adv_data[IBEACON_PAYLOAD_LEN] = {
        0x02, 0x01, 0x06,                         // Flags
        0x1A, 0xFF,                               // Length, Manufacturer Specific
        0x4C, 0x00,                               // Apple Company ID
        0x02, 0x15                                // iBeacon type and length
    };

    memcpy(&adv_data[9], beacon_uuid, IBEACON_UUID_PREFIX_LEN);
    memcpy(&adv_data[21], data[21], IBEACON_DATA_LEN);


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
        printk("iBeacon adv start failed (err %d)\n", err);
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

    struct bt_le_adv_param advp = {
        .id           = BT_ID_DEFAULT,
        .sid          = 0,
        .secondary_max_skip = 0,
        .options      = BT_LE_ADV_OPT_NONE,
        .interval_min = 0x00A0,
        .interval_max = 0x00A0,
        .peer         = NULL,
    };
    err = bt_le_adv_start(&advp, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Adv failed (err %d)\n", err);
    }
    return err;
}

int main(void) {

    double dt = 0.1; // 100 ms timestep

    if (bt_enable(NULL)) {
        printk("BT init failed\n");
        return -1;
    }
    observer_start();

    return 0;
}


