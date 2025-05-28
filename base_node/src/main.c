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

static const uint8_t relay_uuid_base[11] = {
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFB, 0x66, 0x66, 0x66, 0x66, 0x66
};

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *ad)
{
    const uint8_t *data = ad->data;
    uint8_t        len  = ad->len;

    while (len > 1) {
        uint8_t field_len  = data[0];
        uint8_t field_type = data[1];

        if (field_len == 0 || field_len + 1 > len) { break; }

        if (field_type == BT_DATA_MANUFACTURER_DATA && field_len >= 25) {
            const uint8_t *mfg = &data[2];        /* 0x4C 00 02 15 … */

            if (mfg[0] == 0x4C && mfg[1] == 0x00 &&
                mfg[2] == 0x02 && mfg[3] == 0x15) {

                const uint8_t *uuid = &mfg[4];     /* 16-byte UUID   */

                /* Wild-card compare */
                bool match = true;
                for (size_t i = 0; i < sizeof(relay_uuid_base); ++i) {
                    if (relay_uuid_base[i] != uuid[i]) {
                        match = false;
                        break;
                    }
                }

                if (!match) { break; }

                /* ---------------- SENSOR PAYLOAD ---------------- */
                int  sensor_id = (int)uuid[11];                 /* your “node flag” */
                uint16_t temp_raw  = ((uint16_t)uuid[12] << 8) | uuid[13];
                uint8_t  humidity  = uuid[14];
                bool     valid     = uuid[15];

                uint16_t pres_raw  = ((uint16_t)mfg[20] << 8) | mfg[21];
                uint16_t co2_raw   = ((uint16_t)mfg[22] << 8) | mfg[23];

                double   temp_c    = temp_raw / 100.0;         /* assumes ×100 */
                double   pres_hpa  = pres_raw / 10.0;          /* assumes ×10  */

                /* ------------------- PRINT ---------------------- */

                // printk("\nMobile node %d (RSSI %d dB)\n",
                //        sensor_id, rssi);
                // printk("  Temp      : %.2f °C  (raw 0x%04X)\n", temp_c, temp_raw);
                // printk("  Humidity  : %u %%\n",  humidity);
                // printk("  Pressure  : %.1f hPa (raw 0x%04X)\n", pres_hpa, pres_raw);
                // printk("  CO₂       : %u ppm  (raw 0x%04X)\n", co2_raw, co2_raw);
                // printk("  Data valid: %s\n",      valid ? "yes" : "no");
                // printk("-------------------------------------------------\n");
            }
        }

        data += field_len + 1;
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

#if defined(CONFIG_BT_EXT_ADV)
	bt_le_scan_cb_register(&scan_callbacks);
	printk("Registered scan callbacks\n");
#endif /* CONFIG_BT_EXT_ADV */

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