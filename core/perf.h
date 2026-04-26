/*
 * perf.h - shared performance counters and heartbeat publisher.
 *
 * The decode loop calls perf_*() to update counters; a low-priority task
 * publishes EVT_HEARTBEAT at a fixed cadence so all consumers (TUI status
 * line, human log, event stream) see the same numbers.
 */
#ifndef PERF_H
#define PERF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void perf_init(void);
void perf_start_heartbeat_task(const char *app_name, uint32_t period_ms);

void perf_count_bytes(uint32_t n);
void perf_count_msg_good(void);
void perf_count_msg_bad(void);
void perf_set_mag(int avg, int peak);
void perf_set_active_count(int n);

uint32_t perf_get_bytes_per_sec(void);
int      perf_get_msgs_per_sec(void);
int      perf_get_msgs_total(void);
int      perf_get_crc_good(void);
int      perf_get_crc_err(void);
int      perf_get_mag_avg(void);
int      perf_get_mag_peak(void);
int      perf_get_active_count(void);

#ifdef __cplusplus
}
#endif

#endif
