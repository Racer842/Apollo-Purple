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

void lis2dh_thread(void) {
    struct sensor_value accel[3];
    while (1) {
        sensor_sample_fetch(lis2dh);
        sensor_channel_get(lis2dh, SENSOR_CHAN_ACCEL_XYZ, accel);

        double x = sensor_value_to_double(&accel[0]);
		double y = sensor_value_to_double(&accel[1]);
		double z = sensor_value_to_double(&accel[2]);

        if (x < 0.5 && x > -0.5 && y < 0.5 && y > -0.5) {
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