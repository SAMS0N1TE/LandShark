/*
 * diag.h — Phase 0: Telemetry channel on UART1 (GPIO 32 TX / GPIO 33 RX)
 *
 * Replaces the TUI's status display with a machine-parseable, one-line-per-record
 * text stream at 921600 baud. Nothing here blocks — writes go through a small
 * ring buffer drained by a background task.
 *
 * Output format: each line is `TAG ts=<secs.ms> key=val key=val …\r\n`
 * with TAG being one of STAT/DSP/SYM/THR/SYN/BCH/FRM/AUD/OKDMP/FLDMP/SELF/BOOT/ERR.
 *
 * Build: add `diag.c` to CMakeLists.txt. Uses `esp_driver_uart` which is
 * already in PRIV_REQUIRES.
 */
#ifndef DIAG_H
#define DIAG_H

#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Configuration ─────────────────────────────────────────── */
#define DIAG_UART_NUM          1
#define DIAG_UART_TX_PIN       32
#define DIAG_UART_RX_PIN       33
#define DIAG_UART_BAUD         115200
#define DIAG_RING_BYTES        4096
#define DIAG_LINE_MAX          512

/* ── Lifecycle ─────────────────────────────────────────────── */
void diag_init(void);                 /* call once from app_main() */

/* ── Line output ───────────────────────────────────────────── */
/* Non-blocking. Drops on ring-full. Adds "\r\n" itself. */
void diag_line(const char *tag, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));
void diag_vline(const char *tag, const char *fmt, va_list ap);

/* ── 1 Hz status block ─────────────────────────────────────── */
/* Call from any task; internal rate-limit protects against flood. */
void diag_emit_periodic(void);

/* ── Counters updated from hot paths ───────────────────────── */
void diag_count_sync_attempt(int matched_exact, int best_hd);
void diag_count_bch_result(int ok, int ec);    /* ec 0..11 on ok, -1 on fail */
void diag_count_frame(const char *duid_two_chars);

/* ── One-shot NID dibit dump ───────────────────────────────── */
void diag_dump_nid(const char *tag, const int *dibits33, int nac_raw,
                   const char *duid_raw, int ec, int verdict_ok,
                   const char *reason);

/* ── Helpers ───────────────────────────────────────────────── */
float diag_uptime_s(void);

#ifdef __cplusplus
}
#endif

#endif
