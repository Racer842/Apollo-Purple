/*
 * nRF52840-DK application
 * ─────────────────────────────────────────────────────────────
 *  SW0 (button 1) : capture a Grid-EYE 8 × 8 frame
 *  SW1 (button 2) : cycle “Sensor n Selected”
 *  SW2 (button 3) : read soil-moisture voltage on AIN4 / P0.28
 *  SW3 (button 4) : toggle P0.29 (digital OUT) high/low
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>
#include <hal/nrf_saadc.h>          /* for NRF_SAADC_INPUT_AIN4 */

#define PIXELS            64        /* 8 × 8 Grid-EYE frame */
#define SOIL_ADC_CH       4         /* AIN4 ⇨ P0.28 */
#define ADC_RESOLUTION    12
#define ADC_REF_MV        600       /* 0.6 V internal ref */
#define ADC_GAIN          ADC_GAIN_1_6

/* -------- Board buttons ------------------------------------------------------------------ */
static const struct gpio_dt_spec btn1 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios); /* P0.11 */
static const struct gpio_dt_spec btn2 = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios); /* P0.12 */
static const struct gpio_dt_spec btn3 = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios); /* P0.24 */
static const struct gpio_dt_spec btn4 = GPIO_DT_SPEC_GET(DT_ALIAS(sw3), gpios); /* P0.25 */

static struct gpio_callback btn1_cb, btn2_cb, btn3_cb, btn4_cb;

/* -------- Flags set by ISRs -------------------------------------------------------------- */
static volatile bool grab_frame_now;
static volatile bool read_soil_now;
static volatile bool toggle_out_now;

/* -------- Misc state --------------------------------------------------------------------- */
static uint8_t sensor_idx;          /* 0‥3 cycles on button 2 */
static bool out_state;              /* current level on P0.29 */

/* -------- GPIO device handle ------------------------------------------------------------- */
static const struct device *gpio0;

/* -------- SAADC (soil-moisture) ---------------------------------------------------------- */
static const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
static int16_t soil_sample;

static int soil_adc_init(void)
{
    struct adc_channel_cfg ch_cfg = {
        .gain             = ADC_GAIN,
        .reference        = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id       = SOIL_ADC_CH,
#if defined(CONFIG_ADC_NRFX_SAADC)
        .input_positive   = NRF_SAADC_INPUT_AIN4,
#endif
    };
    return adc_channel_setup(adc_dev, &ch_cfg);
}

static int read_soil_mv(int *mv_out)
{
    struct adc_sequence seq = {
        .channels    = BIT(SOIL_ADC_CH),
        .buffer      = &soil_sample,
        .buffer_size = sizeof(soil_sample),
        .resolution  = ADC_RESOLUTION,
    };
    int err = adc_read(adc_dev, &seq);
    if (err) { return err; }

    /* Convert raw sample to millivolts: sample × (Vref / gain) / (2^resolution) */
    int32_t mv = soil_sample;
    mv = (mv * (ADC_REF_MV * 6)) >> ADC_RESOLUTION;   /* gain 1/6 ⇒ FS ≈ 3.6 V */
    *mv_out = (int)mv;
    return 0;
}

/* -------- Button ISRs -------------------------------------------------------------------- */
static void btn1_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{ grab_frame_now = true; }

static void btn2_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    sensor_idx = (sensor_idx + 1) & 0x03;
    printk("Sensor %u Selected\n", sensor_idx + 1);
}

static void btn3_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{ read_soil_now = true; }

static void btn4_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{ toggle_out_now = true; }

/* ----------------------------------------------------------------------------------------- */
void main(void)
{
    const struct device *amg = DEVICE_DT_GET_ANY(panasonic_amg88xx);
    printk("Start\n");

    /* ---------- sanity checks ------------------------------------------------------------ */
    if (!device_is_ready(amg))      { printk("AMG88xx not ready\n"); return; }
    if (!device_is_ready(adc_dev))  { printk("SAADC not ready\n");  return; }
    if (soil_adc_init())            { printk("SAADC channel fail\n"); return; }

    gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio0))    { printk("GPIO0 not ready\n");  return; }

    /* ---------- configure output pin P0.29 ---------------------------------------------- */
    gpio_pin_configure(gpio0, 29, GPIO_OUTPUT_LOW);
    out_state = false;

    /* ---------- configure buttons ------------------------------------------------------- */
    struct {
        const struct gpio_dt_spec *spec;
        struct gpio_callback      *cb;
        gpio_callback_handler_t    isr;
    } buttons[] = {
        { &btn1, &btn1_cb, btn1_isr },
        { &btn2, &btn2_cb, btn2_isr },
        { &btn3, &btn3_cb, btn3_isr },
        { &btn4, &btn4_cb, btn4_isr },
    };

    for (size_t i = 0; i < ARRAY_SIZE(buttons); i++) {
        if (!device_is_ready(buttons[i].spec->port)) {
            printk("Button port %zu not ready\n", i);
            return;
        }
        gpio_pin_configure_dt(buttons[i].spec, GPIO_INPUT);
        gpio_pin_interrupt_configure_dt(buttons[i].spec, GPIO_INT_EDGE_TO_ACTIVE);
        gpio_init_callback(buttons[i].cb, buttons[i].isr, BIT(buttons[i].spec->pin));
        gpio_add_callback(buttons[i].spec->port, buttons[i].cb);
    }

    /* ---------- runtime loop ------------------------------------------------------------ */
    struct sensor_value frame[PIXELS];

    while (1) {
        /* Button 1 – Grid-EYE frame */
        if (grab_frame_now) {
            grab_frame_now = false;
            if (sensor_sample_fetch(amg) == 0 &&
                sensor_channel_get(amg, SENSOR_CHAN_AMBIENT_TEMP, frame) == 0) {

                printk("\nFrame for Sensor %u\n", sensor_idx + 1);
                for (int y = 0; y < 8; y++) {
                    for (int x = 0; x < 8; x++) {
                        printk("%3d ", frame[y*8 + x].val1);
                    }
                    printk("\n");
                }
                printk("\n");
            } else {
                printk("Sensor read failed\n");
            }
        }

        /* Button 3 – soil-moisture voltage */
        if (read_soil_now) {
            read_soil_now = false;
            int mv;
            if (read_soil_mv(&mv) == 0) {
                printk("Soil probe : %d mV\n", mv);
            } else {
                printk("SAADC read error\n");
            }
        }

        /* Button 4 – toggle P0.29 */
        if (toggle_out_now) {
            toggle_out_now = false;
            out_state = !out_state;
            gpio_pin_set(gpio0, 29, out_state);
            printk("P0.29 set %s\n", out_state ? "HIGH" : "LOW");
        }

        k_sleep(K_MSEC(10));
    }
}
