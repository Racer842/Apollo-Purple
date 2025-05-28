/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 /* SPDX-License-Identifier: Apache-2.0 */
 #include <zephyr/kernel.h>
 #include <zephyr/sys/printk.h>
 #include <zephyr/shell/shell.h>
 #include <zephyr/bluetooth/bluetooth.h>
 #include <zephyr/bluetooth/hci.h>
 #include <stdlib.h>
 #include <string.h>
 #include <math.h>
 #include <zephyr/sys/slist.h>
 #include <zephyr/data/json.h>
 #include <stdio.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "sensor_packet.pb.h"

static size_t data_encoded_len;

#define ALPHA 0.25f

static const uint8_t relay_uuid_base[11] = {
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFB, 0x66, 0x66, 0x66, 0x66, 0x66
};

static const uint8_t ultra_sonic[16] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF2, 0xF2
};

struct fused_sample {
    int64_t ts_ms;
    float   temp_c;
    float   hum_pct;
    float   pres_hpa;
    uint16_t co2_ppm;
    bool    valid;
};

struct fused_sample g_fused[4];
struct k_mutex g_fused_mutex;

K_MUTEX_DEFINE(g_fused_mutex);

static void send_fused(uint8_t id)            /* id = 1…4               */
{
    if (id < 1 || id > 4) return;

    struct fused_sample s;
    k_mutex_lock(&g_fused_mutex, K_FOREVER);
    s = g_fused[id-1];                          /* copy under lock        */
    k_mutex_unlock(&g_fused_mutex);

    if (!s.valid) return;                     /* nothing yet            */

    // SensorPacket pkt = {
    //     .sensor_id     = id,
    //     .timestamp_ms  = s.ts_ms,
    //     .temperature   = s.temp_c,
    //     .humidity_pct  = s.hum_pct,
    //     .pressure_hpa  = s.pres_hpa,
    //     .co2_ppm       = s.co2_ppm,
    // };



	SensorData data = SensorData_init_zero;
	data.sensor_id = id;
	//data.timestamp_ms  = s.ts_ms;
    data.temperature   = s.temp_c;
    data.humidity_pct  = s.hum_pct;
    data.pressure_hpa  = s.pres_hpa;
    data.co2_ppm       = s.co2_ppm;

	printk("Struct before encode → "
       "ID:%u T:%.2f°C  H:%.1f%%  P:%.1f hPa  CO₂:%u ppm\n",
       data.sensor_id,
       //(long long)data.timestamp_ms,
       data.temperature,
       data.humidity_pct,
       data.pressure_hpa,
       data.co2_ppm);


    uint8_t buf[64];
    pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));
    if (!pb_encode(&os, SensorData_fields, &data)) {
        printk("PB err: %s\n", PB_GET_ERROR(&os));
        return;
    }

	data_encoded_len = os.bytes_written;
    printk("Nanopb Encoded SensorData (%d bytes): ", data_encoded_len);
    for (size_t i = 0; i < data_encoded_len; i++)
        printk(" %02X", buf[i]);
    printk("\n");
}

static void fusion_update(uint8_t id /*1-4*/,
                          float t, uint8_t h, float p, uint16_t co2)
{
    if (id < 1 || id > 4) return;
    struct fused_sample *s = &g_fused[id-1];

    k_mutex_lock(&g_fused_mutex, K_FOREVER);

    s->ts_ms   = k_uptime_get();
    s->temp_c  = isnan(s->temp_c)  ? t : s->temp_c  * (1-ALPHA) + t * ALPHA;
    s->hum_pct = isnan(s->hum_pct) ? h : s->hum_pct * (1-ALPHA) + h * ALPHA;
    s->pres_hpa= isnan(s->pres_hpa)? p : s->pres_hpa* (1-ALPHA) + p * ALPHA;
    s->co2_ppm = co2;
    s->valid   = true;

    k_mutex_unlock(&g_fused_mutex);

	send_fused(id);
}


static bool uuid_matches(const uint8_t *uuid,
                         const uint8_t *pattern, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        if (pattern[i] != uuid[i]) {
            return false;
        }
    }
    return true;
}

/* --- advertising report callback ------------------------------------------ */
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *ad)
{
    const uint8_t *data = ad->data;
    uint8_t len = ad->len;

    while (len > 1) {
        uint8_t field_len  = data[0];
        uint8_t field_type = data[1];

        if (field_len == 0 || field_len + 1 > len) {
            break;                              /* malformed */
        }

        if (field_type == BT_DATA_MANUFACTURER_DATA && field_len >= 25) {
            const uint8_t *mfg  = &data[2];     /* 4C 00 02 15 …          */

            if (mfg[0] == 0x4C && mfg[1] == 0x00 &&
                mfg[2] == 0x02 && mfg[3] == 0x15) {

                const uint8_t *uuid = &mfg[4];  /* 16-byte UUID           */

                /* -------- Ultrasonic beacon -------------------------------- */
                if (uuid_matches(uuid, ultra_sonic, sizeof(ultra_sonic))) {

                    uint16_t dist_mm = ((uint16_t)mfg[20] << 8) | mfg[21];
                    // printk("Ultrasonic distance: %u mm (%.2f m)  RSSI %d\n",
                    //        dist_mm, dist_mm / 1000.0, rssi);
                    /* continue scanning for other beacons too */
                }

                /* -------- Mobile / relay beacon ---------------------------- */
                if (uuid_matches(uuid, relay_uuid_base,
                                 sizeof(relay_uuid_base))) {

                    /* UUID bytes 12-15  → temp, RH, valid flag               */
                    uint16_t temp_raw = ((uint16_t)uuid[12] << 8) | uuid[13];
                    uint8_t  humidity = uuid[14];
                    bool     valid    = uuid[15];

                    /* Major (mfg[20:21]) → pressure ×10                      */
                    uint16_t pres_raw = ((uint16_t)mfg[20] << 8) | mfg[21];
                    /* Minor (mfg[22:23]) → CO₂ ppm                           */
                    uint16_t co2_raw  = ((uint16_t)mfg[22] << 8) | mfg[23];

                    double temp_c   = temp_raw / 100.0f;
                    double pres_hpa = pres_raw / 10.0f;

                    int sensor_id = uuid[11];    /* node flag you encoded   */

					fusion_update(sensor_id,
                              temp_c,
                              humidity,
                              pres_hpa,
                              co2_raw);

                    // printk("\nMobile node %d  RSSI %d dB\n", sensor_id, rssi);
                    // printk("  Temp      : %.2f °C  (0x%04X)\n", temp_c, temp_raw);
                    // printk("  Humidity  : %u %%\n", humidity);
                    // printk("  Pressure  : %.1f hPa (0x%04X)\n", pres_hpa, pres_raw);
                    // printk("  CO₂       : %u ppm  (0x%04X)\n", co2_raw, co2_raw);
                    // printk("  Data valid: %s\n",   valid ? "yes" : "no");
                    // printk("-------------------------------------------------\n");
                }
            }
        }

        data += field_len + 1;   /* next AD structure */
        len  -= field_len + 1;
    }
}


int observer_start(void)
{
	struct bt_le_scan_param scan_param = {
		.type       = BT_LE_SCAN_TYPE_PASSIVE,
		.options    = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
		.interval   = BT_GAP_SCAN_FAST_INTERVAL,
		.window     = BT_GAP_SCAN_FAST_WINDOW,
	};
	int err;

	err = bt_le_scan_start(&scan_param, device_found);
	if (err) {
		printk("Start scanning failed (err %d)\n", err);
		return err;
	}
	printk("Started scanning...\n");

	return 0;
}

int main(void)
{
	int err;

	printk("Starting Observer Demo\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	(void)observer_start();

	printk("Exiting %s thread.\n", __func__);
	return 0;
}