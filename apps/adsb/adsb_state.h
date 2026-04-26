/*
 * adsb_state.h - canonical ADS-B aircraft state and read accessors.
 *
 * Owns the aircraft tracking table. adsb_decode.c writes; renderers
 * (tui, json) read. No internal/public struct duality - everyone uses
 * adsb_aircraft_t directly.
 */
#ifndef ADSB_STATE_H
#define ADSB_STATE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADSB_MAX_TRACKED 16

typedef struct {
    int     raw_lat;
    int     raw_lon;
    int64_t ts_us;
    bool    valid;
} adsb_cpr_frame_t;

typedef struct {
    uint32_t         icao;
    char             callsign[9];
    int              altitude;
    int              velocity;
    int              heading;
    float            lat;
    float            lon;
    bool             pos_valid;
    int              ew_velocity;
    int              ns_velocity;
    int              vert_rate;
    int              msg_count;
    int64_t          last_seen_us;
    bool             active;

    int64_t          first_seen_us;
    int              good_msg_count;
    int              crc_err_count;
    bool             announced;

    adsb_cpr_frame_t cpr_even;
    adsb_cpr_frame_t cpr_odd;
} adsb_aircraft_t;

void                   adsb_state_init(void);
adsb_aircraft_t       *adsb_state_find_or_create(uint32_t icao);
const adsb_aircraft_t *adsb_state_get(int slot);
int                    adsb_state_active_count(void);
void                   adsb_state_age_out(int64_t now_us, int64_t timeout_us,
                                          void (*on_lost)(adsb_aircraft_t *),
                                          void (*on_late_announce)(adsb_aircraft_t *));

#ifdef __cplusplus
}
#endif

#endif
