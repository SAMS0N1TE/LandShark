#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"

#include "app_registry.h"
#include "event_bus.h"
#include "settings.h"
#include "audio_out.h"
#include "tone.h"
#include "rtl-sdr.h"

#include "p25_state.h"
#include "scanner.h"
#include "diag.h"

p25_state_t       P25 = {0};
scan_state_t      SCAN = {0};
uint32_t          s_tune_freq_hz = 154785000UL;
rtlsdr_dev_t     *rtldev = NULL;
dsp_state_t       s_dsp;
dsd_opts          s_dsd_opts;
dsd_state         s_dsd_state;
dsd_sample_ring_t s_ring;

int autoscan_bch_ok_flag = 0;
int dsd_bch_fail_counter = 0;
volatile int rtl_gain_request = -1;

static const char *TAG = "p25";
static volatile bool s_app_active = false;
static volatile bool s_rx_running = false;
static volatile bool s_dsd_running = false;
static uint8_t *s_iq_buf = NULL;

void sys_log(uint8_t color, const char *fmt, ...)
{
    event_t e = { .kind = EVT_LOG, .ts_us = esp_timer_get_time() };
    strncpy(e.app, "P25", EVT_APP_NAME_MAX);
    int level;
    switch (color) {
        case 4:  level = 3; break;
        case 3:  level = 2; break;
        case 2:  level = 1; break;
        default: level = 0; break;
    }
    e.u.log.level = level;
    strncpy(e.u.log.tag, "p25", sizeof(e.u.log.tag) - 1);
    va_list ap; va_start(ap, fmt);
    vsnprintf(e.u.log.text, sizeof(e.u.log.text), fmt, ap);
    va_end(ap);
    event_bus_publish(&e);
}

void dsd_yield(void) { vTaskDelay(1); }

void audio_beep_request(int kind)
{
    if (!P25.sync_beep_enabled) return;
    if (kind == 1) {
        audio_tone(800.0f,  0.08f, 0.06f);
        audio_tone(1200.0f, 0.08f, 0.06f);
        audio_tone(1600.0f, 0.08f, 0.06f);
    }
}

extern void audio_write_p25_voice(const int16_t *src8k, int n);

static void dsd_decoder_task(void *arg)
{
    esp_task_wdt_add(NULL);
    initOpts(&s_dsd_opts);
    s_dsd_opts.ring = &s_ring;
    s_dsd_opts.verbose = 0;
    s_dsd_opts.errorbars = 1;
    s_dsd_opts.frame_p25p1 = 1;
    s_dsd_opts.mod_c4fm = 1;
    s_dsd_opts.mod_qpsk = 0;
    s_dsd_opts.mod_gfsk = 0;
    s_dsd_opts.mod_threshold = 26;
    s_dsd_opts.ssize = 36;
    s_dsd_opts.msize = 15;
    s_dsd_opts.use_cosine_filter = 1;
    s_dsd_opts.unmute_encrypted_p25 = 1;

    initState(&s_dsd_state);
    s_dsd_state.p25kid = 0;
    if (!s_dsd_state.dibit_buf || !s_dsd_state.audio_out_buf || !s_dsd_state.audio_out_float_buf ||
        !s_dsd_state.cur_mp || !s_dsd_state.prev_mp || !s_dsd_state.prev_mp_enhanced) {
        sys_log(4, "DSD alloc FAILED"); P25.dsd_buffers_ok = false;
        esp_task_wdt_delete(NULL); vTaskDelete(NULL); return;
    }
    P25.dsd_buffers_ok = true;

    static int16_t pcm_buf[2000];
    s_dsd_state.pcm_out_buf = pcm_buf;
    s_dsd_state.pcm_out_size = 2000;
    s_dsd_state.pcm_out_write = 0;
    sys_log(1, "DSD decoder running heap=%lu", (unsigned long)esp_get_free_heap_size());

    s_dsd_running = true;
    while (s_app_active) {
        esp_task_wdt_reset();
        int sync = getFrameSync(&s_dsd_opts, &s_dsd_state);
        if (sync >= 0) {
            P25.dsd_sync_count++;
            P25.dsd_has_sync = true;
            extern int dsp_has_signal_lock;
            dsp_has_signal_lock = 1;
            P25.dsd_nac = s_dsd_state.nac;
            P25.dsd_tg = s_dsd_state.lasttg;
            P25.dsd_src = s_dsd_state.lastsrc;
            snprintf(P25.dsd_ftype, sizeof(P25.dsd_ftype), "%s", s_dsd_state.ftype);
            if (s_dsd_state.rf_mod == 0) strcpy(P25.dsd_modulation, "C4FM");
            else if (s_dsd_state.rf_mod == 1) strcpy(P25.dsd_modulation, "QPSK");
            else strcpy(P25.dsd_modulation, "GFSK");

            esp_task_wdt_reset();
            s_dsd_state.pcm_out_write = 0;
            processFrame(&s_dsd_opts, &s_dsd_state);

            P25.dsd_tg = s_dsd_state.lasttg;
            P25.dsd_src = s_dsd_state.lastsrc;
            snprintf(P25.dsd_fsubtype, sizeof(P25.dsd_fsubtype), "%s", s_dsd_state.fsubtype);
            snprintf(P25.dsd_err_str, sizeof(P25.dsd_err_str), "%s", s_dsd_state.err_str);

            P25.dsd_bch_ok_count = autoscan_bch_ok_flag;
            P25.dsd_bch_fail_count = dsd_bch_fail_counter;
            if (s_dsd_state.nac != 0) P25.dsd_last_ok_nac = s_dsd_state.nac;

            if (s_dsd_state.pcm_out_write > 0) {
                P25.dsd_voice_count++;
                int n = s_dsd_state.pcm_out_write;
                if (n > s_dsd_state.pcm_out_size) n = s_dsd_state.pcm_out_size;
                if (!audio_is_muted()) {
                    audio_write_p25_voice(pcm_buf, n);
                    sys_log(2, "VOX %d smp errs=%u", n, s_dsd_state.debug_audio_errors);
                }
            }
            sys_log(1, "SYNC %s nac=%04X tg=%d %s",
                    P25.dsd_modulation, P25.dsd_nac, P25.dsd_tg, P25.dsd_fsubtype);
        } else {
            P25.dsd_has_sync = false;
            extern int dsp_has_signal_lock;
            dsp_has_signal_lock = 0;
        }
    }
    esp_task_wdt_delete(NULL);
    s_dsd_running = false;
    vTaskDelete(NULL);
}

static void p25_rx_task(void *arg)
{
    dsp_init(&s_dsp);
    dsp_set_mode(&s_dsp, DEMOD_C4FM);
    dsp_set_gain(&s_dsp, P25.demod_gain);
    sys_log(1, "DSP mode: C4FM");
    memset(&s_ring, 0, sizeof(s_ring));
    P25.ring_size = DSD_SAMPLE_RING_SIZE;

    s_iq_buf = malloc(P25_USB_BUF_LENGTH);
    if (!s_iq_buf) { sys_log(4, "OOM rx buffer"); vTaskDelete(NULL); return; }
    sys_log(1, "RX buf OK %d bytes heap=%lu",
            P25_USB_BUF_LENGTH, (unsigned long)esp_get_free_heap_size());

    static int16_t audio_buf[8192];

    xTaskCreatePinnedToCore(dsd_decoder_task, "dsd_decode", 16384, NULL, 5, NULL, 1);
    sys_log(1, "IQ read loop starting");

    int n_read = 0, read_errors = 0;
    uint32_t iq_bucket = 0, audio_bucket = 0;
    int64_t stats_ts = esp_timer_get_time();

    s_rx_running = true;

    while (s_app_active) {
        if (SCAN.request_scan) { SCAN.request_scan = false; scanner_run_sweep(); }
        if (SCAN.request_tune && SCAN.peak_freq > 0) {
            SCAN.request_tune = false;
            s_tune_freq_hz = SCAN.peak_freq;
            rtlsdr_set_center_freq(rtldev, s_tune_freq_hz);
            rtlsdr_reset_buffer(rtldev);
            vTaskDelay(pdMS_TO_TICKS(10));
            sys_log(1, "Tuned: %.4f MHz", s_tune_freq_hz / 1e6);
        }

        if (rtl_gain_request >= 0) {
            int gain = rtl_gain_request;
            rtl_gain_request = -1;
            P25.rtl_gain_tenths = gain;
            if (rtldev) {
                rtlsdr_set_tuner_gain_mode(rtldev, gain == 0 ? 0 : 1);
                if (gain > 0) rtlsdr_set_tuner_gain(rtldev, gain);
            }
            sys_log(1, "RTL gain: %.1f dB%s", gain / 10.0f, gain == 0 ? " (AGC)" : "");
        }

        if (s_dsp.mode == DEMOD_FSK4_TRACKING) {
            static int64_t last_sync_us = 0;
            static int reset_check_count = 0;
            if (P25.dsd_has_sync) last_sync_us = esp_timer_get_time();
            if (++reset_check_count >= 100) {
                reset_check_count = 0;
                int64_t now_us = esp_timer_get_time();
                if ((now_us - last_sync_us) >= 3000000LL)
                    dsp_fsk4_reset_tracker(&s_dsp);
            }
        }

        bool full = true;
        for (int i = 0; i < P25_USB_BUF_LENGTH; i += P25_USB_PACKET_SIZE) {
            int r = rtlsdr_read_sync(rtldev, &s_iq_buf[i], P25_USB_PACKET_SIZE, &n_read);
            if (r < 0) {
                full = false; read_errors++;
                if (read_errors <= 3) sys_log(4, "USB err r=%d", r);
                vTaskDelay(pdMS_TO_TICKS(10));
                break;
            }
            if ((uint32_t)n_read < P25_USB_PACKET_SIZE) { full = false; break; }
        }

        if (full) {
            read_errors = 0;
            iq_bucket += P25_USB_BUF_LENGTH;

            {
                static int64_t last_iqr_us = 0;
                int64_t tnow_iqr = esp_timer_get_time();
                if (tnow_iqr - last_iqr_us > 1000000LL) {
                    last_iqr_us = tnow_iqr;
                    uint32_t sum_i = 0, sum_q = 0;
                    uint64_t sumsq = 0;
                    uint8_t  umin = 255, umax = 0;
                    int      peak_dev = 0;
                    const int half_n = P25_USB_BUF_LENGTH / 2;
                    for (int i = 0; i < P25_USB_BUF_LENGTH; i += 2) {
                        uint8_t ui = s_iq_buf[i];
                        uint8_t uq = s_iq_buf[i + 1];
                        if (ui < umin) umin = ui;
                        if (ui > umax) umax = ui;
                        if (uq < umin) umin = uq;
                        if (uq > umax) umax = uq;
                        sum_i += ui;
                        sum_q += uq;
                        int di = (int)ui - 128;
                        int dq = (int)uq - 128;
                        int adi = di < 0 ? -di : di;
                        int adq = dq < 0 ? -dq : dq;
                        if (adi > peak_dev) peak_dev = adi;
                        if (adq > peak_dev) peak_dev = adq;
                        sumsq += (uint64_t)(di * di) + (uint64_t)(dq * dq);
                    }
                    float mean_i = (float)sum_i / (float)half_n - 128.0f;
                    float mean_q = (float)sum_q / (float)half_n - 128.0f;
                    float ms = (float)sumsq / (float)(2 * half_n);
                    float rms_norm = sqrtf(ms) / 127.5f;
                    float peak_norm = (float)peak_dev / 127.5f;
                    P25.iq_level = peak_norm;

                    int rms_mmm  = (int)(rms_norm  * 1000.0f + 0.5f);
                    int peak_mmm = (int)(peak_norm * 1000.0f + 0.5f);
                    diag_line("IQR",
                        "umin=%u umax=%u peak=%d.%03d rms=%d.%03d "
                        "mean_i=%+.2f mean_q=%+.2f n=%d",
                        (unsigned)umin, (unsigned)umax,
                        peak_mmm / 1000, peak_mmm % 1000,
                        rms_mmm  / 1000, rms_mmm  % 1000,
                        (double)mean_i, (double)mean_q, half_n);
                }
            }

            int na = dsp_process_iq(&s_dsp, s_iq_buf, P25_USB_BUF_LENGTH, audio_buf, 8192);
            audio_bucket += na;

            for (int i = 0; i < na; i++) {
                int next = (s_ring.write_idx + 1) % DSD_SAMPLE_RING_SIZE;
                if (next != s_ring.read_idx) {
                    s_ring.buf[s_ring.write_idx] = audio_buf[i];
                    s_ring.write_idx = next;
                }
            }
        }

        P25.read_errors = read_errors;
        P25.ring_fill = dsd_ring_available(&s_ring);

        int64_t now = esp_timer_get_time();
        if (now - stats_ts > 1000000LL) {
            P25.iq_bytes_sec = iq_bucket;
            P25.audio_samples_sec = audio_bucket;
            P25.iq_bytes_total += iq_bucket;
            iq_bucket = 0; audio_bucket = 0; stats_ts = now;
        }

        if (read_errors > 20) {
            sys_log(4, "Read errors, pause 2s");
            vTaskDelay(pdMS_TO_TICKS(2000));
            read_errors = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    for (int i = 0; i < 200 && s_dsd_running; i++) vTaskDelay(pdMS_TO_TICKS(10));
    free(s_iq_buf);
    s_iq_buf = NULL;
    s_rx_running = false;
    vTaskDelete(NULL);
}

static void p25_on_enter(void)
{
    if (s_app_active) return;
    s_app_active = true;

    P25.demod_gain = -9000.0f;
    P25.rtl_gain_tenths = RTL_DEFAULT_GAIN;
    P25.sync_beep_enabled = false;

    scanner_init();

    extern rtlsdr_dev_t *rtlsdr_dev_get(void);
    rtldev = rtlsdr_dev_get();

    if (rtldev) {
        rtlsdr_set_center_freq(rtldev, s_tune_freq_hz);
        rtlsdr_set_sample_rate(rtldev, RTL_SAMPLE_RATE);
        rtlsdr_set_tuner_gain_mode(rtldev, P25.rtl_gain_tenths == 0 ? 0 : 1);
        if (P25.rtl_gain_tenths > 0) rtlsdr_set_tuner_gain(rtldev, P25.rtl_gain_tenths);
        rtlsdr_set_tuner_bandwidth(rtldev, 0);
        rtlsdr_reset_buffer(rtldev);
        sys_log(1, "Radio: %.4f MHz %dkSPS", s_tune_freq_hz / 1e6, RTL_SAMPLE_RATE / 1000);
    }

    xTaskCreatePinnedToCore(p25_rx_task, "p25_rx", 16384, NULL, 5, NULL, 1);
}

static void p25_on_exit(void)
{
    s_app_active = false;
    for (int i = 0; i < 300 && (s_rx_running || s_dsd_running); i++)
        vTaskDelay(pdMS_TO_TICKS(10));
    rtldev = NULL;
}

static void p25_on_sample(uint8_t *iq, int len)
{
    (void)iq; (void)len;
}

extern void p25_draw_main(int top, int rows, int cols);
extern void p25_on_key(tui_key_t k);

static const app_t P25_APP = {
    .name         = "P25",
    .default_freq = 154785000UL,
    .default_rate = RTL_SAMPLE_RATE,
    .default_gain = RTL_DEFAULT_GAIN,
    .on_enter     = p25_on_enter,
    .on_exit      = p25_on_exit,
    .on_sample    = p25_on_sample,
#ifdef CONFIG_ENABLE_TUI
    .draw_main    = p25_draw_main,
    .on_key       = p25_on_key,
#else
    .draw_main    = NULL,
    .on_key       = NULL,
#endif
};

int p25_app_register(void)
{
    return app_register(&P25_APP);
}
