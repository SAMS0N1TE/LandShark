/*
 * event_stream.h - machine-readable JSONL event output.
 *
 * Each event becomes one JSON object on its own line, prefixed with the
 * ASCII record-separator byte 0x1E (RS) so a host SBC reading the UART
 * can demultiplex our frames from any human ESP_LOG output sharing the
 * same UART. Standard JSONL parsers ignore the RS prefix; tools that
 * grep for "{" find everything cleanly.
 */
#ifndef EVENT_STREAM_H
#define EVENT_STREAM_H

#ifdef __cplusplus
extern "C" {
#endif

void event_stream_init(void);

#ifdef __cplusplus
}
#endif

#endif
