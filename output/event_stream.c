/* event_stream.c - JSONL formatter, one record-separated line per event,
 * emitted on a dedicated UART so the consumer never sees TUI escape codes
 * or ESP_LOG lines. UART0 (the main serial port) is owned by the TUI and
 * the system console; printing JSONL there would interleave with screen
 * redraws and log output, making the stream unparseable.
 *
 * UART2 / TX = GPIO 17, 115200 8N1. Wire that pin to the host's RX and
 * read directly. Each record is prefixed with 0x1E (ASCII RS) and ends
 * with 0x0A (LF) so a reader can resync mid-stream if it joins late. */

#include "event_stream.h"
#include "event_bus.h"
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "driver/uart.h"
#include "esp_log.h"

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

#define RS "\x1e"

static bool s_uart_ready = false;

static void emit_str(const char *buf, int len)
{
    if (!s_uart_ready || len <= 0) return;
    uart_write_bytes(STREAM_UART_NUM, buf, len);
}

static void emit_contact(const event_t *e)
{
    char buf[STREAM_LINE_MAX];
    const evt_contact_t *c = &e->u.contact;
    int n = snprintf(buf, sizeof(buf),
        RS "{\"t\":%lld,\"k\":\"%s\",\"app\":\"%s\","
        "\"icao\":\"%06lX\",\"cs\":\"%s\","
        "\"alt\":%d,\"vel\":%d,\"hdg\":%d,\"vs\":%d,"
        "\"lat\":%.5f,\"lon\":%.5f,\"pos\":%s,\"shaky\":%s}\n",
        (long long)e->ts_us, evt_kind_name(e->kind), e->app,
        (unsigned long)c->icao, c->callsign,
        c->altitude, c->velocity, c->heading, c->vert_rate,
        c->lat, c->lon,
        c->pos_valid ? "true" : "false",
        c->crc_shaky ? "true" : "false");
    emit_str(buf, n);
}

static void emit_heartbeat(const event_t *e)
{
    char buf[STREAM_LINE_MAX];
    const evt_heartbeat_t *h = &e->u.hb;
    int n = snprintf(buf, sizeof(buf),
        RS "{\"t\":%lld,\"k\":\"%s\",\"app\":\"%s\","
        "\"bps\":%lu,\"msgs\":%d,\"mps\":%d,"
        "\"crc_good\":%d,\"crc_err\":%d,\"ac\":%d,"
        "\"mag_avg\":%d,\"mag_peak\":%d}\n",
        (long long)e->ts_us, evt_kind_name(e->kind), e->app,
        (unsigned long)h->bytes_per_sec, h->msgs_total, h->msgs_per_sec,
        h->crc_good, h->crc_err, h->active_count,
        h->mag_avg, h->mag_peak);
    emit_str(buf, n);
}

static void emit_app_switch(const event_t *e)
{
    char buf[STREAM_LINE_MAX];
    int n = snprintf(buf, sizeof(buf),
        RS "{\"t\":%lld,\"k\":\"%s\",\"from\":\"%s\",\"to\":\"%s\"}\n",
        (long long)e->ts_us, evt_kind_name(e->kind),
        e->u.sw.from, e->u.sw.to);
    emit_str(buf, n);
}

static void emit_simple(const event_t *e)
{
    char buf[STREAM_LINE_MAX];
    int n = snprintf(buf, sizeof(buf),
        RS "{\"t\":%lld,\"k\":\"%s\",\"app\":\"%s\"}\n",
        (long long)e->ts_us, evt_kind_name(e->kind), e->app);
    emit_str(buf, n);
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
         * consumer can wait for it before processing further events. */
        char banner[128];
        int n = snprintf(banner, sizeof(banner),
            "\x1e{\"k\":\"stream_init\",\"uart\":%d,\"tx_gpio\":%d,\"baud\":%d}\n",
            STREAM_UART_NUM, STREAM_UART_TX_PIN, STREAM_UART_BAUD);
        uart_write_bytes(STREAM_UART_NUM, banner, n);

        ESP_LOGI("event_stream",
                 "JSONL on UART%d TX=GPIO%d %d 8N1",
                 STREAM_UART_NUM, STREAM_UART_TX_PIN, STREAM_UART_BAUD);
    }

    event_bus_subscribe(on_event, NULL);
}
