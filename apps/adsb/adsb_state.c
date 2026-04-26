/* adsb_state.c - aircraft table, find/create, aging. */

#include "adsb_state.h"
#include "esp_timer.h"
#include <string.h>

#define GATE_WINDOW_US (10 * 1000000LL)

static adsb_aircraft_t s_aircraft[ADSB_MAX_TRACKED];

void adsb_state_init(void)
{
    memset(s_aircraft, 0, sizeof(s_aircraft));
}

adsb_aircraft_t *adsb_state_find_or_create(uint32_t icao)
{
    adsb_aircraft_t *empty = NULL;
    for (int i = 0; i < ADSB_MAX_TRACKED; i++) {
        if (s_aircraft[i].active && s_aircraft[i].icao == icao)
            return &s_aircraft[i];
        if (!s_aircraft[i].active && !empty)
            empty = &s_aircraft[i];
    }
    if (empty) {
        memset(empty, 0, sizeof(*empty));
        empty->icao          = icao;
        empty->active        = true;
        empty->first_seen_us = esp_timer_get_time();
    }
    return empty;
}

const adsb_aircraft_t *adsb_state_get(int slot)
{
    if (slot < 0 || slot >= ADSB_MAX_TRACKED) return NULL;
    return &s_aircraft[slot];
}

int adsb_state_active_count(void)
{
    int n = 0;
    for (int i = 0; i < ADSB_MAX_TRACKED; i++)
        if (s_aircraft[i].active) n++;
    return n;
}

void adsb_state_age_out(int64_t now_us, int64_t timeout_us,
                        void (*on_lost)(adsb_aircraft_t *),
                        void (*on_late_announce)(adsb_aircraft_t *))
{
    for (int i = 0; i < ADSB_MAX_TRACKED; i++) {
        adsb_aircraft_t *a = &s_aircraft[i];
        if (!a->active) continue;

        if (!a->announced && a->good_msg_count >= 1 &&
            (now_us - a->first_seen_us) > GATE_WINDOW_US) {
            a->announced = true;
            if (on_late_announce) on_late_announce(a);
        }

        if (now_us - a->last_seen_us > timeout_us) {
            if (on_lost) on_lost(a);
            a->active = false;
        }
    }
}
