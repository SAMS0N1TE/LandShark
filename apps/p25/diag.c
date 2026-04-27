/*
 * diag.c — Phase 0 telemetry on UART1.
 *
 * Design notes:
 *   - UART1 TX on GPIO 32, RX reserved on GPIO 33. RX is unused today but
 *     kept mapped so we can add a simple command interface later (freq set,
 *     pause, etc.) without reflashing.
 *   - Non-blocking producer / blocking-drain consumer. Producers call
 *     diag_line() from any context (including the dsp_pipeline hot path);
 *     lines are memcpy'd into a ring and a single background task drains it
 *     to the UART TX FIFO. Drops are counted, not catastrophic.
 *   - Counters are plain int32 with atomic reads via a short critical
 *     section. The 1 Hz snapshot reads + zeros them in one go.
 *   - Deliberately no ANSI escapes, no cursor moves. This is a log.
 */
#include "diag.h"
#include "p25_state.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>

static const char *TAG = "DIAG";

/* ── private state ── */
static RingbufHandle_t s_diag_ring   = NULL;
static SemaphoreHandle_t s_lock = NULL;   /* protects counters below */
static TaskHandle_t s_task      = NULL;
static int64_t s_start_us       = 0;
static int64_t s_last_periodic_us = 0;
static uint32_t s_dropped       = 0;

/* Counter snapshot, updated at 1 Hz and zeroed after emit. */
typedef struct {
    int sync_attempts;
    int sync_exact;           /* Hamming-distance 0 matches */
    int sync_hd1;
    int sync_hd2;
    int sync_hd3;
    int best_hd_this_period;  /* smallest HD seen this period */

    int bch_ok;
    int bch_fail;
    int bch_ec_hist[12];      /* index 0..11 is error-count bucket */

    int frm_hdu;
    int frm_ldu1;
    int frm_ldu2;
    int frm_tdu;
    int frm_tdulc;
    int frm_tsdu;
    int frm_pdu;
    int frm_other;
} counters_t;

static counters_t s_cnt;

/* ── UART TX drain task ── */
static void diag_tx_task(void *arg)
{
    (void)arg;
    while (1) {
        size_t len = 0;
        void *p = xRingbufferReceive(s_diag_ring, &len, pdMS_TO_TICKS(200));
        if (p) {
            uart_write_bytes(DIAG_UART_NUM, (const char *)p, len);
            vRingbufferReturnItem(s_diag_ring, p);
        }
    }
}

/* ── init ── */
void diag_init(void)
{
    s_start_us = esp_timer_get_time();
    s_last_periodic_us = s_start_us;
    memset(&s_cnt, 0, sizeof(s_cnt));
    s_cnt.best_hd_this_period = 99;

    s_lock = xSemaphoreCreateMutex();
    s_diag_ring = xRingbufferCreate(DIAG_RING_BYTES, RINGBUF_TYPE_NOSPLIT);
    if (!s_diag_ring || !s_lock) {
        ESP_LOGE(TAG, "diag buffer alloc failed");
        return;
    }

    /* UART1 driver install. TX queue 0 — we drain via ring buffer ourselves. */
    const uart_config_t cfg = {
        .baud_rate  = DIAG_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(DIAG_UART_NUM, 1024, 4096, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(DIAG_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(DIAG_UART_NUM,
                                 DIAG_UART_TX_PIN, DIAG_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    /* Pin drain task to core 1 so it never fights the USB host on core 0. */
    xTaskCreatePinnedToCore(diag_tx_task, "diag_tx", 3072, NULL,
                            tskIDLE_PRIORITY + 2, &s_task, 1);

    /* Banner so you can confirm the UART is alive the moment you connect. */
    diag_line("BOOT", "diag_init ok uart=%d tx=%d rx=%d baud=%d",
              DIAG_UART_NUM, DIAG_UART_TX_PIN, DIAG_UART_RX_PIN, DIAG_UART_BAUD);
    diag_line("BOOT", "phase=1 target_freq=154785000 expect=P25p1_NH");
}

/* ── time ── */
float diag_uptime_s(void)
{
    return (float)((esp_timer_get_time() - s_start_us) / 1000) / 1000.0f;
}

/* ── core emit ── */
void diag_vline(const char *tag, const char *fmt, va_list ap)
{
    /* Format the message body once into a scratch buffer; we use it for
     * both the diag UART (with timestamp prefix) and the TUI mirror. */
    char body[DIAG_LINE_MAX];
    int  m = vsnprintf(body, sizeof(body), fmt, ap);
    if (m < 0) return;
    if (m >= (int)sizeof(body)) m = (int)sizeof(body) - 1;

    /* Diag UART path. */
    if (s_diag_ring) {
        char line[DIAG_LINE_MAX];
        int n = snprintf(line, sizeof(line), "%-4s ts=%.3f ",
                         tag ? tag : "?", diag_uptime_s());
        if (n > 0 && n < (int)sizeof(line) - 2) {
            int copy = m;
            if (n + copy > (int)sizeof(line) - 2) copy = (int)sizeof(line) - 2 - n;
            memcpy(line + n, body, copy);
            line[n + copy]     = '\r';
            line[n + copy + 1] = '\n';
            BaseType_t ok = xRingbufferSend(s_diag_ring, line, n + copy + 2, 0);
            if (ok != pdTRUE) s_dropped++;
        }
    }

    /* TUI mirror: selected tags only, so the user can see frame-sync
     * hunt diagnostics on the LOG page without hooking up the diag UART
     * (UART1 GPIO 32). HUNT shows the dibit window vs the expected sync
     * pattern. SHD shows the best Hamming distance per hunt period. NID
     * shows the raw NAC/DUID dibit dump. PERIOD is the periodic counter
     * rollup. IQR is per-second IQ level / DC / RMS. */
    if (tag && (
            !strcmp(tag, "HUNT")   ||
            !strcmp(tag, "SHD")    ||
            !strcmp(tag, "NID")    ||
            !strcmp(tag, "PERIOD") ||
            !strcmp(tag, "IQR"))) {
        extern void sys_log(uint8_t color, const char *fmt, ...);
        sys_log(0, "%s %s", tag, body);
    }
}

void diag_line(const char *tag, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    diag_vline(tag, fmt, ap);
    va_end(ap);
}

/* ── counters ── */
void diag_count_sync_attempt(int matched_exact, int best_hd)
{
    if (!s_lock) return;
    if (xSemaphoreTake(s_lock, 0) != pdTRUE) return;  /* skip on contention */
    s_cnt.sync_attempts++;
    if (matched_exact) s_cnt.sync_exact++;
    else if (best_hd == 1) s_cnt.sync_hd1++;
    else if (best_hd == 2) s_cnt.sync_hd2++;
    else if (best_hd == 3) s_cnt.sync_hd3++;
    if (best_hd < s_cnt.best_hd_this_period) s_cnt.best_hd_this_period = best_hd;
    xSemaphoreGive(s_lock);
}

void diag_count_bch_result(int ok, int ec)
{
    if (!s_lock) return;
    if (xSemaphoreTake(s_lock, 0) != pdTRUE) return;
    if (ok) {
        s_cnt.bch_ok++;
        if (ec >= 0 && ec < 12) s_cnt.bch_ec_hist[ec]++;
    } else {
        s_cnt.bch_fail++;
    }
    xSemaphoreGive(s_lock);
}

void diag_count_frame(const char *duid2)
{
    if (!s_lock || !duid2) return;
    if (xSemaphoreTake(s_lock, 0) != pdTRUE) return;
    if      (!strcmp(duid2, "00")) s_cnt.frm_hdu++;
    else if (!strcmp(duid2, "11")) s_cnt.frm_ldu1++;
    else if (!strcmp(duid2, "22")) s_cnt.frm_ldu2++;
    else if (!strcmp(duid2, "03")) s_cnt.frm_tdu++;
    else if (!strcmp(duid2, "33")) s_cnt.frm_tdulc++;
    else if (!strcmp(duid2, "13")) s_cnt.frm_tsdu++;
    else if (!strcmp(duid2, "30")) s_cnt.frm_pdu++;
    else                           s_cnt.frm_other++;
    xSemaphoreGive(s_lock);
}

/* ── NID dibit dump ── */
void diag_dump_nid(const char *tag, const int *dibits33, int nac_raw,
                   const char *duid_raw, int ec, int verdict_ok,
                   const char *reason)
{
    if (!dibits33) return;
    /* Build one compact line. Dibits 0..5 are NAC, 6..7 are DUID, 8..32 are BCH+parity. */
    char buf[320];
    int n = snprintf(buf, sizeof(buf),
                     "nac_raw=%03X duid_raw=%s ec=%d verdict=%s",
                     nac_raw & 0xFFF,
                     duid_raw ? duid_raw : "??",
                     ec,
                     verdict_ok ? "OK" : "FAIL");
    if (reason && *reason) {
        n += snprintf(buf + n, sizeof(buf) - n, " reason=%s", reason);
    }
    n += snprintf(buf + n, sizeof(buf) - n, " dibits=[");
    for (int i = 0; i < 33 && n < (int)sizeof(buf) - 6; i++) {
        n += snprintf(buf + n, sizeof(buf) - n, "%d%s", dibits33[i] & 3,
                      (i == 32) ? "" : ",");
        if (i == 5 || i == 7) {  /* visual separators after NAC / after DUID */
            n += snprintf(buf + n, sizeof(buf) - n, "|");
        }
    }
    if (n < (int)sizeof(buf) - 1) buf[n++] = ']';
    buf[n] = 0;
    diag_line(tag, "%s", buf);
}

/* ── 1 Hz periodic snapshot ── */
void diag_emit_periodic(void)
{
    int64_t now = esp_timer_get_time();
    if (now - s_last_periodic_us < 1000000) return;  /* 1 Hz cap */
    s_last_periodic_us = now;

    counters_t snap;
    if (s_lock && xSemaphoreTake(s_lock, pdMS_TO_TICKS(2)) == pdTRUE) {
        snap = s_cnt;
        memset(&s_cnt, 0, sizeof(s_cnt));
        s_cnt.best_hd_this_period = 99;
        xSemaphoreGive(s_lock);
    } else {
        return;
    }

    const char *mode_str = "??";
    switch (s_dsp.mode) {
        case DEMOD_C4FM:          mode_str = "C4FM"; break;
        case DEMOD_CQPSK:         mode_str = "CQPSK"; break;
        case DEMOD_DIFF_4FSK:     mode_str = "DIFF4FSK"; break;
        case DEMOD_FSK4_TRACKING: mode_str = "FSK4T"; break;
        default:                  mode_str = "?"; break;
    }

    diag_line("STAT", "freq=%lu iq_B=%lu iq_Bs=%lu ring=%d/%d rd_err=%d lvl=%.2f",
              (unsigned long)s_tune_freq_hz,
              (unsigned long)P25.iq_bytes_total,
              (unsigned long)P25.iq_bytes_sec,
              P25.ring_fill, P25.ring_size,
              P25.read_errors,
              (double)P25.iq_level);
    diag_line("DSP",  "mode=%s agc_gain=%.2f demod_gain=%.2f rtl_gain=%.1f dc_i=%+.4f dc_q=%+.4f",
              mode_str,
              (double)s_dsp.agc_gain,
              (double)P25.demod_gain,
              (double)P25.rtl_gain_tenths / 10.0,
              (double)s_dsp.dc_avg_i,
              (double)s_dsp.dc_avg_q);
    diag_line("SYN",  "att=%d hd0=%d hd1=%d hd2=%d hd3=%d best=%d",
              snap.sync_attempts, snap.sync_exact,
              snap.sync_hd1, snap.sync_hd2, snap.sync_hd3,
              (snap.best_hd_this_period == 99) ? -1 : snap.best_hd_this_period);
    diag_line("BCH",  "ok=%d fail=%d ec=[0:%d,1:%d,2:%d,3:%d,4:%d,5:%d,6:%d,7:%d,8:%d,9:%d,10:%d,11:%d]",
              snap.bch_ok, snap.bch_fail,
              snap.bch_ec_hist[0], snap.bch_ec_hist[1], snap.bch_ec_hist[2],
              snap.bch_ec_hist[3], snap.bch_ec_hist[4], snap.bch_ec_hist[5],
              snap.bch_ec_hist[6], snap.bch_ec_hist[7], snap.bch_ec_hist[8],
              snap.bch_ec_hist[9], snap.bch_ec_hist[10], snap.bch_ec_hist[11]);
    diag_line("FRM",  "hdu=%d ldu1=%d ldu2=%d tdu=%d tdulc=%d tsdu=%d pdu=%d other=%d",
              snap.frm_hdu, snap.frm_ldu1, snap.frm_ldu2, snap.frm_tdu,
              snap.frm_tdulc, snap.frm_tsdu, snap.frm_pdu, snap.frm_other);
    if (s_dropped) {
        diag_line("ERR", "diag_drops=%u", (unsigned)s_dropped);
        s_dropped = 0;
    }
}
