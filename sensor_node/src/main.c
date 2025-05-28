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
#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/sys/printk.h>

#define LED_RED_NODE    DT_ALIAS(led0)
#define LED_GREEN_NODE  DT_ALIAS(led1)
#define LED_BLUE_NODE   DT_ALIAS(led2)

LOG_MODULE_REGISTER(sensor_bt, LOG_LEVEL_INF);

/* Sensor devices */
const struct device *hts221 = DEVICE_DT_GET_ONE(st_hts221);
const struct device *lps22hb = DEVICE_DT_GET_ONE(st_lps22hb_press);
const struct device *ccs811 = DEVICE_DT_GET_ONE(ams_ccs811);

/* GPIO LEDs */
static const struct gpio_dt_spec led_red = GPIO_DT_SPEC_GET(LED_RED_NODE, gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(LED_GREEN_NODE, gpios);
static const struct gpio_dt_spec led_blue = GPIO_DT_SPEC_GET(LED_BLUE_NODE, gpios);

/* Thread configuration */
#define STACK_SIZE 1024
#define PRIORITY  5

/* Bluetooth configuration */
#define APPLE_COMPANY_ID    0x004C
#define APPLE_BEACON_TYPE   0x02
#define APPLE_BEACON_LENGTH 0x15

/* Global sensor data structure */
struct sensor_data {
    uint16_t temperature;    // scaled by 100 (25.5C = 2550)
    uint8_t humidity;        // 0-100%
    uint16_t pressure;       // scaled by 10 (1013.25 hPa = 10132)
    uint8_t tvoc;           // scaled TVOC value
    uint16_t co2;           // CO2 in ppm
    bool data_valid;        // Flag to indicate if data is valid
} current_sensor_data = {0};

/* Thread synchronization */
K_MUTEX_DEFINE(sensor_data_mutex);

/* Preset UUID configurations */
struct uuid_preset {
    uint8_t bytes[4];
    const char *name;
};

static const struct uuid_preset uuid_presets[] = {
    {{0xAA, 0x01, 0x01, 0x01}, "Location A"},
    {{0xBB, 0x02, 0x02, 0x02}, "Location B"},
    {{0xCC, 0x03, 0x03, 0x03}, "Location C"},
    {{0xDD, 0x04, 0x04, 0x04}, "Location D"}
};

/* Default UUID - will be modified by CLI */
static uint8_t beacon_uuid[16] = {
    0xAA, 0xAA, 0xAA, 0xAA,  // First 4 bytes - node identifier (preset 1)
    0xFF, 0xFB, 0x48, 0xD2,
    0xB0, 0x60, 0xD0, 0xF5,
    0xA7, 0x10, 0x96, 0xE0
};

/* Node ID (first 4 bytes of UUID) */
static uint32_t node_id = 0xAA010101;

/* Bluetooth advertisement data */
static uint8_t ibeacon_data[] = {
    /* Flags */
    0x02, 0x01, 0x06,
    
    /* Apple iBeacon */
    0x1A, 0xFF,             /* Length and type of manufacturer specific data */
    0x4C, 0x00,             /* Apple company identifier (little-endian) */
    0x02,                   /* iBeacon type */
    0x15,                   /* iBeacon prefix length */
    
    /* UUID (16 bytes) - will be updated */
    0xAA, 0x01, 0x01, 0x01, 0xFF, 0xFB, 0x48, 0xD2,
    0xB0, 0x60, 0xD0, 0xF5, 0xA7, 0x10, 0x96, 0xE0,
    
    /* Major (2 bytes) - sensor data type indicator */
    0x00, 0x01,
    
    /* Minor (2 bytes) - sensor data payload */
    0x00, 0x00,
    
    /* Measured RSSI at 1 meter distance */
    0xC8  /* -56 dBm */
};

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, &ibeacon_data[5], 25)
};

/* CCS811 state tracking */
static bool ccs811_warming_up = true;
static int64_t ccs811_start_time;

/* LED control function */
void set_rgb_color(bool r, bool g, bool b) {
    gpio_pin_set_dt(&led_red, r);
    gpio_pin_set_dt(&led_green, g);
    gpio_pin_set_dt(&led_blue, b);
}

/* Update beacon UUID */
void update_beacon_uuid(void) {
    /* Update the UUID in iBeacon data */
    memcpy(&ibeacon_data[9], beacon_uuid, 16);
}

/* Pack sensor data into beacon UUID and Major/Minor */
void pack_sensor_data_to_beacon(void) {
    k_mutex_lock(&sensor_data_mutex, K_FOREVER);
    
    // Pack sensor data into the LOWEST bytes of UUID (bytes 12-15)
    // This keeps the higher bytes cleaner for identification
    beacon_uuid[12] = current_sensor_data.temperature >> 8;         // 0x09
    beacon_uuid[13] = current_sensor_data.temperature & 0xFF;       // 0x60

    beacon_uuid[14] = current_sensor_data.humidity;                   // Humidity
    beacon_uuid[15] = current_sensor_data.data_valid ? 0x01 : 0x00;   // Data validity flag
    
    // Update UUID in beacon data
    memcpy(&ibeacon_data[9], beacon_uuid, 16);
    
    // Pack remaining sensor data into Major/Minor fields (4 bytes total)
    // Major (2 bytes): Pressure data
    ibeacon_data[25] = (current_sensor_data.pressure >> 8);  // Pressure high
    ibeacon_data[26] = current_sensor_data.pressure & 0xFF;         // Pressure low
    
    // Minor (2 bytes): Air quality data  
    ibeacon_data[27] = (current_sensor_data.co2 >> 8);                    // TVOC
    ibeacon_data[28] = (current_sensor_data.co2 & 0xFF);      // CO2 high byte only (0-255 * 256 range)
    
    k_mutex_unlock(&sensor_data_mutex);
}

/* Validate sensor readings */
bool validate_sensor_reading(double temp, double humidity, double pressure) {
    // Basic sanity checks for sensor readings
    if (temp < -40.0 || temp > 85.0) return false;        // Temperature range
    if (humidity < 0.0 || humidity > 100.0) return false;  // Humidity range
    if (pressure < 50.0 || pressure > 1200.0) return false; // Pressure range (hPa)
    return true;
}

/* Single consolidated sensor reading thread */
void sensor_thread(void) {
    struct sensor_value temp, humidity, pressure, tvoc, co2;
    int consecutive_failures = 0;
    
    // Record CCS811 startup time
    ccs811_start_time = k_uptime_get();
    
    while (1) {
        bool sensor_updated = false;
        bool reading_valid = false;
        
        k_mutex_lock(&sensor_data_mutex, K_FOREVER);
        
        // Read HTS221 (Temperature & Humidity)
        if (device_is_ready(hts221)) {
            if (sensor_sample_fetch(hts221) == 0) {
                sensor_channel_get(hts221, SENSOR_CHAN_AMBIENT_TEMP, &temp);
                sensor_channel_get(hts221, SENSOR_CHAN_HUMIDITY, &humidity);
                
                double temp_c = sensor_value_to_double(&temp);
                double humid_pct = sensor_value_to_double(&humidity);
                
                // Validate readings
                if (temp_c >= -40.0 && temp_c <= 85.0 && humid_pct >= 0.0 && humid_pct <= 100.0) {
                    // Scale and store: temp * 100, humidity 0-100
                    current_sensor_data.temperature = (uint16_t)(temp_c * 100);
                    current_sensor_data.humidity = (uint8_t)(humid_pct);
                    sensor_updated = true;
                    reading_valid = true;
            
                }
            } else {
                printk("Failed to read HTS221\n");
            }
        }
        
        // Read LPS22HB (Pressure)
        if (device_is_ready(lps22hb)) {
            if (sensor_sample_fetch(lps22hb) == 0) {
                sensor_channel_get(lps22hb, SENSOR_CHAN_PRESS, &pressure);
                
                double press_pa = sensor_value_to_double(&pressure);

                // Validate pressure reading (reasonable atmospheric pressure range)
                if (press_pa >= 50.0 && press_pa <= 1200.0) {
                    // Scale and store: pressure * 10 (1013.25 hPa = 10132)
                    current_sensor_data.pressure = (uint16_t)(press_pa * 10);
                    sensor_updated = true;
                    reading_valid = true;
                    
                }
            } else {
                printk("Failed to read LPS22HB\n");
            }
        }
        
        // Read CCS811 (Air Quality) - only after warm-up period
        int64_t uptime = k_uptime_get();
        if (ccs811_warming_up && (uptime - ccs811_start_time) > 5000) { // 5 second warm-up
            ccs811_warming_up = false;
            printk("CCS811 warm-up complete\n");
        }
        
        if (device_is_ready(ccs811) && !ccs811_warming_up) {
            if (sensor_sample_fetch(ccs811) == 0) {
                sensor_channel_get(ccs811, SENSOR_CHAN_CO2, &co2);
                sensor_channel_get(ccs811, SENSOR_CHAN_VOC, &tvoc);
                
                double tvoc_val = sensor_value_to_double(&tvoc);
                double co2_val = sensor_value_to_double(&co2);

                // Basic validation for air quality readings
                if (tvoc_val >= 0 && co2_val >= 400 && co2_val <= 8192) {
                    // Scale TVOC to fit in uint8_t (divide by 10, cap at 255)
                    current_sensor_data.tvoc = (uint8_t)MIN(tvoc_val / 10, 255);
                    current_sensor_data.co2 = (uint16_t)MIN(co2_val, 65535);
                    sensor_updated = true;
                    reading_valid = true;
                    
                }
            } else {
                printk("Failed to read CCS811\n");
            }
        } else if (ccs811_warming_up) {
            printk("CCS811 warming up... %lld seconds remaining\n", 
                   (20000 - (uptime - ccs811_start_time)) / 1000);
        }
        
        // Update data validity flag
        if (reading_valid) {
            current_sensor_data.data_valid = true;
            consecutive_failures = 0;
        } else {
            consecutive_failures++;
            if (consecutive_failures > 10) {
                current_sensor_data.data_valid = false;
            }
        }
        
        k_mutex_unlock(&sensor_data_mutex);
        
        // Update beacon data if any sensor was read successfully
        if (sensor_updated) {
            pack_sensor_data_to_beacon();
            // LED indication based on data validity
            if (reading_valid) {
                set_rgb_color(0, 1, 0); // Green for valid data
            } else {
                set_rgb_color(1, 1, 0); // Yellow for questionable data
            }
            k_msleep(50);
            set_rgb_color(0, 0, 0);
        } else {
            // Red flash for no sensor readings
            set_rgb_color(1, 0, 0);
            k_msleep(50);
            set_rgb_color(0, 0, 0);
        }
        
        k_msleep(1000); // Read all sensors every 1 second (reduced frequency)
    }
}

/* Bluetooth advertising update thread */
void bt_update_thread(void) {
    int err;
    int retry_count = 0;
    
    while (1) {
        // Update advertising data with current sensor values
        err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
        if (err) {
            retry_count++;
            if (retry_count % 10 == 0) { // Only print error every 10 failures
                printk("Advertising data update failed (err %d), retry %d\n", err, retry_count);
            }
            k_msleep(500); // Wait longer on error
        } else {
            printk("Sent\n");
            retry_count = 0; // Reset on success
            k_msleep(250); // Update advertising data every 250ms
        }
    }
}

/* Thread definitions */
K_THREAD_DEFINE(sensor_tid, STACK_SIZE, sensor_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(bt_update_tid, STACK_SIZE, bt_update_thread, NULL, NULL, NULL, 2, 0, 0);

/* CLI Commands */
static int cmd_set_preset(const struct shell *shell, size_t argc, char **argv) {
    if (argc != 2) {
        shell_error(shell, "Usage: -s <preset_number>");
        shell_print(shell, "Available presets:");
        for (int i = 0; i < ARRAY_SIZE(uuid_presets); i++) {
            shell_print(shell, "  %d: %s (%02X %02X %02X %02X)", 
                       i + 1, uuid_presets[i].name,
                       uuid_presets[i].bytes[0], uuid_presets[i].bytes[1],
                       uuid_presets[i].bytes[2], uuid_presets[i].bytes[3]);
        }
        return -EINVAL;
    }
    
    int preset_num = atoi(argv[1]);
    if (preset_num < 1 || preset_num > ARRAY_SIZE(uuid_presets)) {
        shell_error(shell, "Preset number must be between 1 and %d", ARRAY_SIZE(uuid_presets));
        return -EINVAL;
    }
    
    // Set UUID to selected preset
    int preset_idx = preset_num - 1;
    memcpy(beacon_uuid, uuid_presets[preset_idx].bytes, 4);
    
    node_id = (beacon_uuid[0] << 24) | (beacon_uuid[1] << 16) | 
              (beacon_uuid[2] << 8) | beacon_uuid[3];
    
    update_beacon_uuid();
    
    shell_print(shell, "UUID set to preset %d: %s", preset_num, uuid_presets[preset_idx].name);
    shell_print(shell, "UUID bytes: %02X %02X %02X %02X", 
                beacon_uuid[0], beacon_uuid[1], beacon_uuid[2], beacon_uuid[3]);
    
    // Flash blue LED to indicate configuration change
    set_rgb_color(0, 0, 1);
    k_msleep(200);
    set_rgb_color(0, 0, 0);
    
    return 0;
}

static int cmd_show_status(const struct shell *shell, size_t argc, char **argv) {
    k_mutex_lock(&sensor_data_mutex, K_FOREVER);
    
    shell_print(shell, "=== Thingy52 Sensor Node Status ===");
    shell_print(shell, "Node ID: 0x%08X", node_id);
    shell_print(shell, "UUID: %02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X",
                beacon_uuid[0], beacon_uuid[1], beacon_uuid[2], beacon_uuid[3],
                beacon_uuid[4], beacon_uuid[5], beacon_uuid[6], beacon_uuid[7],
                beacon_uuid[8], beacon_uuid[9], beacon_uuid[10], beacon_uuid[11],
                beacon_uuid[12], beacon_uuid[13], beacon_uuid[14], beacon_uuid[15]);
    
    // Convert scaled values back to readable format
    double temp_c = (double)current_sensor_data.temperature / 100.0;
    double press_hpa = (double)current_sensor_data.pressure / 10.0;
    
    shell_print(shell, "Data Valid: %s", current_sensor_data.data_valid ? "YES" : "NO");
    shell_print(shell, "Temperature: %.2f°C (raw: %u)", temp_c, current_sensor_data.temperature);
    shell_print(shell, "Humidity: %u%%", current_sensor_data.humidity);
    shell_print(shell, "Pressure: %.1f hPa (raw: %u)", press_hpa, current_sensor_data.pressure);
    shell_print(shell, "TVOC: %u (scaled)", current_sensor_data.tvoc);
    shell_print(shell, "CO2: %u ppm", current_sensor_data.co2);
    
    shell_print(shell, "CCS811 Status: %s", ccs811_warming_up ? "Warming up" : "Ready");
    
    shell_print(shell, "\n=== Data Packing ===");
    shell_print(shell, "UUID bytes 12-15: Temp + Humidity + Valid flag");
    shell_print(shell, "Major (bytes 25-26): Pressure data");
    shell_print(shell, "Minor (bytes 27-28): TVOC + CO2 high byte");
    
    k_mutex_unlock(&sensor_data_mutex);
    return 0;
}

static int cmd_list_presets(const struct shell *shell, size_t argc, char **argv) {
    shell_print(shell, "Available UUID presets:");
    for (int i = 0; i < ARRAY_SIZE(uuid_presets); i++) {
        shell_print(shell, "  %d: %s (%02X %02X %02X %02X)", 
                   i + 1, uuid_presets[i].name,
                   uuid_presets[i].bytes[0], uuid_presets[i].bytes[1],
                   uuid_presets[i].bytes[2], uuid_presets[i].bytes[3]);
    }
    shell_print(shell, "Use '-s <number>' to select a preset");
    return 0;
}

static int cmd_reset_ccs811(const struct shell *shell, size_t argc, char **argv) {
    shell_print(shell, "Resetting CCS811 warm-up timer...");
    ccs811_warming_up = true;
    ccs811_start_time = k_uptime_get();
    shell_print(shell, "CCS811 will be ready in 20 seconds");
    return 0;
}

/* Shell command registration */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_thingy,
    SHELL_CMD_ARG(-s, NULL, "Set UUID preset (1-4)", cmd_set_preset, 2, 0),
    SHELL_CMD(status, NULL, "Show current status", cmd_show_status),
    SHELL_CMD(presets, NULL, "List available presets", cmd_list_presets),
    SHELL_CMD(reset_ccs811, NULL, "Reset CCS811 warm-up timer", cmd_reset_ccs811),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(thingy, &sub_thingy, "Thingy52 sensor node commands", NULL);

/* Main function */
int main(void) {
    int err;
    
    printk("Starting Thingy52 Sensor + Bluetooth Node v2.0\n");
    
    /* Initialize GPIOs */
    if (!gpio_is_ready_dt(&led_red) || !gpio_is_ready_dt(&led_green) || !gpio_is_ready_dt(&led_blue)) {
        printk("LED GPIO devices not ready\n");
        return 0;
    }
    
    gpio_pin_configure_dt(&led_red, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led_blue, GPIO_OUTPUT_ACTIVE);
    
    /* Initial LED state - red during initialization */
    set_rgb_color(1, 0, 0);
    
    /* Check sensor devices */
    if (!device_is_ready(hts221)) {
        printk("HTS221 device not ready\n");
    } else {
        printk("HTS221 device ready\n");
    }
    
    if (!device_is_ready(lps22hb)) {
        printk("LPS22HB device not ready\n");
    } else {
        printk("LPS22HB device ready\n");
    }
    
    if (!device_is_ready(ccs811)) {
        printk("CCS811 device not ready\n");
    } else {
        printk("CCS811 device ready (warming up for 20s)\n");
    }
    
    /* Initialize Bluetooth */
    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        // Continue without bluetooth
    } else {
        printk("Bluetooth initialized\n");
        
        /* Update initial beacon UUID */
        update_beacon_uuid();
        
        /* Start advertising */
        struct bt_le_adv_param adv_param = {
            .id = BT_ID_DEFAULT,
            .sid = 0,
            .secondary_max_skip = 0,
            .options = BT_LE_ADV_OPT_USE_NAME,
            .interval_min = 0x0020,   // 32 ms
            .interval_max = 0x0040,   // 64 ms
            .peer = NULL,
        };
        
        err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
        if (err) {
            printk("Advertising failed to start (err %d)\n", err);
        } else {
            printk("Advertising started\n");
        }
    }
    
    LOG_INF("Thingy52 Sensor + Bluetooth Node Initialized.");
    printk("Use 'thingy -s <1-4>' to set UUID preset\n");
    printk("Use 'thingy status' to view current sensor data\n");
    printk("Use 'thingy presets' to list available presets\n");
    printk("Use 'thingy reset_ccs811' to reset air quality sensor\n");
    
    return 0;
}