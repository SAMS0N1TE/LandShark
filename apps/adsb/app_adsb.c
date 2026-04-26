/* app_adsb.c - ADS-B app registration and periodic-age driver. */

#include "app_registry.h"
#include "adsb_decode.h"
#include "adsb_state.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#ifdef CONFIG_ENABLE_TUI
extern void adsb_draw_main(int top, int rows, int cols);
extern void adsb_on_enter_tui(void);
#endif

static void age_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        adsb_periodic_age(esp_timer_get_time());
    }
}

static void adsb_on_enter(void)
{
#ifdef CONFIG_ENABLE_TUI
    adsb_on_enter_tui();
#endif
}
static void adsb_on_exit(void) { }

static const app_t ADSB_APP = {
    .name         = "ADS-B",
    .default_freq = 1090000000UL,
    .default_rate = 2000000,
    .default_gain = 496,
    .on_enter     = adsb_on_enter,
    .on_exit      = adsb_on_exit,
    .on_sample    = adsb_on_sample,
#ifdef CONFIG_ENABLE_TUI
    .draw_main    = adsb_draw_main,
#else
    .draw_main    = NULL,
#endif
    .on_key       = NULL,
};

const app_t *adsb_app_desc(void) { return &ADSB_APP; }

int adsb_app_register(void)
{
    adsb_decode_init();
    xTaskCreatePinnedToCore(age_task, "adsb_age", 3072, NULL, 1, NULL, 1);
    return app_register(&ADSB_APP);
}
