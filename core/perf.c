/* perf.c - global counters and heartbeat publisher. */

#include "perf.h"
#include "event_bus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "perf";

static volatile uint32_t s_bytes_window  = 0;
static volatile uint32_t s_bytes_per_sec = 0;
static volatile int      s_msgs_total    = 0;
static volatile int      s_msgs_window   = 0;
static volatile int      s_msgs_per_sec  = 0;
static volatile int      s_crc_good      = 0;
static volatile int      s_crc_err       = 0;
static volatile int      s_mag_avg       = 0;
static volatile int      s_mag_peak      = 0;
static volatile int      s_active        = 0;
static int64_t           s_window_start  = 0;

void perf_init(void)
{
    s_window_start  = esp_timer_get_time();
    s_bytes_window  = 0;
    s_bytes_per_sec = 0;
    s_msgs_window   = 0;
    s_msgs_per_sec  = 0;
    s_msgs_total    = 0;
    s_crc_good      = 0;
    s_crc_err       = 0;
    s_mag_avg       = 0;
    s_mag_peak      = 0;
    s_active        = 0;
}

static void roll_window_if_needed(void)
{
    int64_t now = esp_timer_get_time();
    int64_t dt  = now - s_window_start;
    if (dt < 1000000) return;
    s_bytes_per_sec = (uint32_t)((s_bytes_window * 1000000ULL) / (uint64_t)dt);
    s_msgs_per_sec  = (int)((int64_t)s_msgs_window * 1000000LL / dt);
    s_bytes_window  = 0;
    s_msgs_window   = 0;
    s_window_start  = now;
}

void perf_count_bytes(uint32_t n)    { s_bytes_window += n; roll_window_if_needed(); }
void perf_count_msg_good(void)       { s_crc_good++;  s_msgs_total++; s_msgs_window++; }
void perf_count_msg_bad(void)        { s_crc_err++; }
void perf_set_mag(int avg, int peak) { s_mag_avg = avg; s_mag_peak = peak; }
void perf_set_active_count(int n)    { s_active = n; }

uint32_t perf_get_bytes_per_sec(void) { roll_window_if_needed(); return s_bytes_per_sec; }
int  perf_get_msgs_per_sec(void)      { roll_window_if_needed(); return s_msgs_per_sec; }
int  perf_get_msgs_total(void)        { return s_msgs_total; }
int  perf_get_crc_good(void)          { return s_crc_good; }
int  perf_get_crc_err(void)           { return s_crc_err; }
int  perf_get_mag_avg(void)           { return s_mag_avg; }
int  perf_get_mag_peak(void)          { return s_mag_peak; }
int  perf_get_active_count(void)      { return s_active; }

typedef struct {
    char     app_name[16];
    uint32_t period_ms;
} hb_ctx_t;

static void hb_task(void *arg)
{
    hb_ctx_t *ctx = (hb_ctx_t *)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(ctx->period_ms));
        evt_heartbeat_t hb = {
            .bytes_per_sec = perf_get_bytes_per_sec(),
            .msgs_total    = perf_get_msgs_total(),
            .msgs_per_sec  = perf_get_msgs_per_sec(),
            .crc_good      = perf_get_crc_good(),
            .crc_err       = perf_get_crc_err(),
            .mag_avg       = perf_get_mag_avg(),
            .mag_peak      = perf_get_mag_peak(),
            .active_count  = perf_get_active_count(),
        };
        event_bus_publish_heartbeat(ctx->app_name, &hb);
    }
}

void perf_start_heartbeat_task(const char *app_name, uint32_t period_ms)
{
    hb_ctx_t *ctx = pvPortMalloc(sizeof(hb_ctx_t));
    if (!ctx) {
        ESP_LOGW(TAG, "hb_ctx alloc failed");
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
    if (app_name) strncpy(ctx->app_name, app_name, sizeof(ctx->app_name) - 1);
    ctx->period_ms = period_ms ? period_ms : 30000;
    xTaskCreatePinnedToCore(hb_task, "perf_hb", 3072, ctx, 1, NULL, 0);
}
