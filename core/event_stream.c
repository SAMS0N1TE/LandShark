/* event_stream.c - JSONL formatter, one record-separated line per event,
 * emitted on a dedicated UART so the consumer never sees TUI escape codes
 * or ESP_LOG lines. UART0 (the main serial port) is owned by the TUI and
 * the system console; printing JSONL there would interleave with screen
 * redraws and log output, making the stream unparseable.
 *
 * UART2 / TX = configurable via Kconfig (default GPIO 3), 115200 8N1.
 * Wire that pin to the host's RX and read directly. Each record is
 * framed as: 0x1E (ASCII RS) ... JSON body ... 0x0A (LF). The RS
 * prefix lets a reader resync mid-stream if it joins late, and the LF
 * terminates the record so line-oriented parsers (jq -c, NDJSON tools)
 * also work.
 *
 * Concurrency model:
 *   - event_bus already serialises subscriber callbacks under its own
 *     mutex, so on_event() runs single-threaded with respect to other
 *     bus subscribers. We take a *second* mutex around every actual
 *     uart_write_bytes() call so that:
 *       (a) the boot banner can't splice into the first heartbeat if
 *           init races with an early publish,
 *       (b) any future direct-emit caller (a CLI, a watchdog dump,
 *           etc.) can't interleave a partial line with an in-flight
 *           bus emit, and
 *       (c) the framing bytes (RS, JSON body, LF) for a single record
 *           are guaranteed contiguous on the wire even if the body
 *           write blocks on a full TX buffer.
 *     Defence in depth - cheap, and it directly addresses the
 *     "frame truncated mid-string then next frame slammed in with no
 *     separator" symptom seen in v9 captures.
 *
 * Framing robustness:
 *   - RS (0x1E) and LF (0x0A) are written as separate, explicit byte
 *     sequences around the JSON body rather than embedded inside the
 *     snprintf format string. snprintf is not actually the failure
 *     mode here, but pulling the framing bytes out of the format
 *     string makes them impossible to lose to a future refactor that
 *     truncates the body buffer. The body write and the trailing LF
 *     write happen under the same mutex hold, so they can't be
 *     separated by another writer. */

#include "event_stream.h"
#include "event_bus.h"
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifndef CONFIG_OUTPUT_EVENT_STREAM_UART_NUM
#define CONFIG_OUTPUT_EVENT_STREAM_UART_NUM  2
#endif
#ifndef CONFIG_OUTPUT_EVENT_STREAM_TX_GPIO
#define CONFIG_OUTPUT_EVENT_STREAM_TX_GPIO   3
#endif

#define STREAM_UART_NUM     CONFIG_OUTPUT_EVENT_STREAM_UART_NUM
#define STREAM_UART_TX_PIN  CONFIG_OUTPUT_EVENT_STREAM_TX_GPIO
#define STREAM_UART_BAUD    115200
#define STREAM_LINE_MAX     384

/* Framing bytes pulled out of format strings so they can never be
 * accidentally truncated, escaped, or stripped by an intermediate
 * processing step. */
static const char FRAME_RS[1] = { 0x1E };
static const char FRAME_LF[1] = { 0x0A };

static bool s_uart_ready = false;

/* Device-level write mutex. See concurrency notes at top of file. */
static SemaphoreHandle_t s_tx_mux = NULL;

/* Write a single record: RS prefix, body, LF terminator. The three
 * writes happen under the same mutex hold so the record is atomic
 * with respect to any other emit_record() caller. uart_write_bytes()
 * blocks if the TX ring buffer is full, but blocking inside the lock
 * is fine - other emitters wait, the wire drains, we proceed. */
static void emit_record(const char *body, int body_len)
{
    if (!s_uart_ready || body_len <= 0) return;

    if (s_tx_mux) xSemaphoreTake(s_tx_mux, portMAX_DELAY);

    uart_write_bytes(STREAM_UART_NUM, FRAME_RS, sizeof(FRAME_RS));
    uart_write_bytes(STREAM_UART_NUM, body,     body_len);
    uart_write_bytes(STREAM_UART_NUM, FRAME_LF, sizeof(FRAME_LF));

    if (s_tx_mux) xSemaphoreGive(s_tx_mux);
}

static void emit_contact(const event_t *e)
{
    char buf[STREAM_LINE_MAX];
    const evt_contact_t *c = &e->u.contact;
    /* Body only - framing is added by emit_record(). */
    int n = snprintf(buf, sizeof(buf),
        "{\"t\":%lld,\"k\":\"%s\",\"app\":\"%s\","
        "\"icao\":\"%06lX\",\"cs\":\"%s\","
        "\"alt\":%d,\"vel\":%d,\"hdg\":%d,\"vs\":%d,"
        "\"lat\":%.5f,\"lon\":%.5f,\"pos\":%s,\"shaky\":%s}",
        (long long)e->ts_us, evt_kind_name(e->kind), e->app,
        (unsigned long)c->icao, c->callsign,
        c->altitude, c->velocity, c->heading, c->vert_rate,
        c->lat, c->lon,
        c->pos_valid ? "true" : "false",
        c->crc_shaky ? "true" : "false");
    if (n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;
    emit_record(buf, n);
}

static void emit_heartbeat(const event_t *e)
{
    char buf[STREAM_LINE_MAX];
    const evt_heartbeat_t *h = &e->u.hb;
    int n = snprintf(buf, sizeof(buf),
        "{\"t\":%lld,\"k\":\"%s\",\"app\":\"%s\","
        "\"bps\":%lu,\"msgs\":%d,\"mps\":%d,"
        "\"crc_good\":%d,\"crc_err\":%d,\"ac\":%d,"
        "\"mag_avg\":%d,\"mag_peak\":%d}",
        (long long)e->ts_us, evt_kind_name(e->kind), e->app,
        (unsigned long)h->bytes_per_sec, h->msgs_total, h->msgs_per_sec,
        h->crc_good, h->crc_err, h->active_count,
        h->mag_avg, h->mag_peak);
    if (n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;
    emit_record(buf, n);
}

static void emit_app_switch(const event_t *e)
{
    char buf[STREAM_LINE_MAX];
    int n = snprintf(buf, sizeof(buf),
        "{\"t\":%lld,\"k\":\"%s\",\"from\":\"%s\",\"to\":\"%s\"}",
        (long long)e->ts_us, evt_kind_name(e->kind),
        e->u.sw.from, e->u.sw.to);
    if (n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;
    emit_record(buf, n);
}

static void emit_simple(const event_t *e)
{
    char buf[STREAM_LINE_MAX];
    int n = snprintf(buf, sizeof(buf),
        "{\"t\":%lld,\"k\":\"%s\",\"app\":\"%s\"}",
        (long long)e->ts_us, evt_kind_name(e->kind), e->app);
    if (n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;
    emit_record(buf, n);
}

static void on_event(const event_t *e, void *user)
{
    (void)user;
    if (!e) return;
    switch (e->kind) {
    case EVT_CONTACT_NEW:
    case EVT_CONTACT_CONFIRMED:
    case EVT_CONTACT_LOST:
    case EVT_CONTACT_POSITION:
    case EVT_CONTACT_ALTITUDE:
    case EVT_CONTACT_VELOCITY:
    case EVT_CONTACT_IDENT:
        emit_contact(e);
        break;
    case EVT_HEARTBEAT:
        emit_heartbeat(e);
        break;
    case EVT_APP_SWITCHED:
        emit_app_switch(e);
        break;
    case EVT_BOOT:
    case EVT_SHUTDOWN:
    case EVT_DEVICE_ATTACHED:
    case EVT_DEVICE_DETACHED:
    case EVT_TUNER_LOCKED:
        emit_simple(e);
        break;
    case EVT_LOG:
    case EVT_DECODE_ERROR:
    default:
        break;
    }
}

void event_stream_init(void)
{
    /* Mutex first - emit_record() checks for it but it must exist
     * before any subscriber callback can fire. Created idempotently. */
    if (!s_tx_mux) {
        s_tx_mux = xSemaphoreCreateMutex();
        if (!s_tx_mux) {
            ESP_LOGE("event_stream", "tx mutex create failed");
            /* Continue without mutex - bus serialisation alone covers
             * the common case; we just lose defence in depth. */
        }
    }

    if (!uart_is_driver_installed(STREAM_UART_NUM)) {
        const uart_config_t cfg = {
            .baud_rate  = STREAM_UART_BAUD,
            .data_bits  = UART_DATA_8_BITS,
            .parity     = UART_PARITY_DISABLE,
            .stop_bits  = UART_STOP_BITS_1,
            .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        if (uart_driver_install(STREAM_UART_NUM, 256, 4096, 0, NULL, 0) == ESP_OK) {
            uart_param_config(STREAM_UART_NUM, &cfg);
            uart_set_pin(STREAM_UART_NUM,
                         STREAM_UART_TX_PIN,
                         UART_PIN_NO_CHANGE,
                         UART_PIN_NO_CHANGE,
                         UART_PIN_NO_CHANGE);
            s_uart_ready = true;
        } else {
            ESP_LOGE("event_stream",
                     "uart_driver_install UART%d failed", STREAM_UART_NUM);
        }
    } else {
        s_uart_ready = true;
    }

    if (s_uart_ready) {
        /* Banner: confirms the wire is hot the moment the host opens
         * the port. Also gives a parseable "I'm alive" line so the
         * consumer can wait for it before processing further events.
         * Goes through emit_record() so it gets the same RS/LF
         * framing and mutex protection as every other line - meaning
         * the banner can never splice into a heartbeat that fires
         * during init. Kind name is uppercase to match the schema
         * convention used by every other event. */
        char banner[128];
        int n = snprintf(banner, sizeof(banner),
            "{\"k\":\"STREAM_INIT\",\"uart\":%d,\"tx_gpio\":%d,\"baud\":%d}",
            STREAM_UART_NUM, STREAM_UART_TX_PIN, STREAM_UART_BAUD);
        if (n >= (int)sizeof(banner)) n = (int)sizeof(banner) - 1;
        emit_record(banner, n);

        ESP_LOGI("event_stream",
                 "JSONL on UART%d TX=GPIO%d %d 8N1",
                 STREAM_UART_NUM, STREAM_UART_TX_PIN, STREAM_UART_BAUD);
    }

    event_bus_subscribe(on_event, NULL);
}
