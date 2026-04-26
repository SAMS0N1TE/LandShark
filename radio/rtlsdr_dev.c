/* rtlsdr_dev.c - RTL-SDR device init and runtime tuning.
 *
 * The setup task runs OUTSIDE the USB host event callback - control
 * transfers issued during init need the host event loop to process their
 * completions, and that loop is blocked while the callback runs. Doing
 * the setup synchronously from the callback deadlocks. */

#include "rtlsdr_dev.h"
#include "rtl-sdr.h"
#include "app_registry.h"
#include "settings.h"
#include "event_bus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

extern void adsb_rx_task(void *arg);

static const char *TAG = "rtlsdr_dev";

static rtlsdr_dev_t *s_dev = NULL;

typedef struct {
    uint8_t                  dev_addr;
    usb_host_client_handle_t client;
} setup_arg_t;

static void rtlsdr_setup_task(void *arg)
{
    setup_arg_t *sa = (setup_arg_t *)arg;

    int r = rtlsdr_open(&s_dev, sa->dev_addr, sa->client);
    if (r < 0) {
        ESP_LOGE(TAG, "rtlsdr_open failed");
        vPortFree(sa);
        vTaskDelete(NULL);
        return;
    }

    const app_t *cur = app_current();
    uint32_t freq = cur ? settings_get_freq(cur) : 1090000000UL;
    uint32_t rate = cur ? cur->default_rate     : 2000000UL;
    int      gain = cur ? settings_get_gain(cur) : 496;

    rtlsdr_set_center_freq(s_dev, freq);
    rtlsdr_set_sample_rate(s_dev, rate);

    /* Manual gain at the tuner max plus digital AGC on - dump1090's
     * working ADS-B recipe. The "auto" tuner mode locks VGA at 26.5 dB
     * which is too low for distant 1090 MHz squitter bursts. */
    rtlsdr_set_tuner_gain_mode(s_dev, 1);
    rtlsdr_set_tuner_gain(s_dev, gain);
    rtlsdr_set_agc_mode(s_dev, 1);
    rtlsdr_set_freq_correction(s_dev, 0);

    rtlsdr_reset_buffer(s_dev);

    event_bus_publish_simple(EVT_TUNER_LOCKED, cur ? cur->name : "rtlsdr");
    ESP_LOGI(TAG, "tuner locked  %lu Hz  %lu sps  gain=%d",
             (unsigned long)freq, (unsigned long)rate, gain);

    xTaskCreatePinnedToCore(adsb_rx_task, "adsb_rx", 16384, NULL, 5, NULL, 1);

    vPortFree(sa);
    vTaskDelete(NULL);
}

void rtlsdr_dev_setup_async(uint8_t dev_addr, usb_host_client_handle_t client)
{
    setup_arg_t *sa = pvPortMalloc(sizeof(*sa));
    if (!sa) { ESP_LOGE(TAG, "setup arg alloc failed"); return; }
    sa->dev_addr = dev_addr;
    sa->client   = client;
    xTaskCreatePinnedToCore(rtlsdr_setup_task, "rtlsdr_setup", 8192,
                            sa, 4, NULL, 0);
}

uint32_t rtlsdr_dev_set_freq(uint32_t hz)
{
    if (!s_dev) return 0;
    rtlsdr_set_center_freq(s_dev, hz);
    return hz;
}
int rtlsdr_dev_set_gain(int tenths_db)
{
    if (!s_dev) return -1;
    return rtlsdr_set_tuner_gain(s_dev, tenths_db);
}
int rtlsdr_dev_set_sample_rate(uint32_t hz)
{
    if (!s_dev) return -1;
    return rtlsdr_set_sample_rate(s_dev, hz);
}

rtlsdr_dev_t *rtlsdr_dev_get(void) { return s_dev; }
