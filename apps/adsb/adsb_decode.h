/*
 * adsb_decode.h - ADS-B sample-to-message pipeline.
 *
 * The on_sample entry takes raw IQ from the radio stream, computes
 * magnitudes, runs Mode-S detection, and dispatches each decoded
 * message into the aircraft table + audio events + event bus.
 */
#ifndef ADSB_DECODE_H
#define ADSB_DECODE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void adsb_decode_init(void);

void adsb_on_sample(uint8_t *iq, int len);

void adsb_periodic_age(int64_t now_us);

#ifdef __cplusplus
}
#endif

#endif
