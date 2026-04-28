/* adsb_render.c - TUI plane table for the ADS-B app's PAGE_MAIN.
 * Compiled only when CONFIG_ENABLE_TUI is set. */

#include "tui.h"
#include "adsb_state.h"
#include "perf.h"
#include <stdio.h>
#include <string.h>

void adsb_on_enter_tui(void)
{
    tui_waterfall_reset();
}

void adsb_draw_main(int top, int rows, int cols)
{
    tui_goto(top, 3);
    printf(C_LABEL "AIRCRAFT " RESET C_BRIGHT "%d" RESET
           C_DIM "   " RESET
           C_LABEL "MSGS " RESET C_BRIGHT "%d" RESET
           C_DIM " (%d/s)   " RESET
           C_LABEL "CRC " RESET C_GREEN "%d" RESET C_DIM "/" RESET C_RED "%d" RESET EL,
           perf_get_active_count(), perf_get_msgs_total(), perf_get_msgs_per_sec(),
           perf_get_crc_good(), perf_get_crc_err());

    tui_goto(top + 2, 3);
    printf(C_LABEL "%-8s %-9s %8s %6s %5s %9s %9s %6s %5s" RESET EL,
           "ICAO", "CALLSIGN", "ALT ft", "KT", "HDG",
           "LAT", "LON", "V/S", "MSGS");

    tui_hline(top + 3, 3, cols - 2);

    /* Pre-clear the plane-table area: when an aircraft ages out, its row
     * number is abandoned by the active-only render loop below, so without
     * this pre-clear the old text stays on screen forever. */
    for (int r = top + 4; r < top + rows - 1; r++) {
        tui_goto(r, 3);
        fputs(EL, stdout);
    }

    int body_row = top + 4;
    int body_max = top + rows - 1;
    int shown    = 0;

    for (int i = 0; i < ADSB_MAX_TRACKED && body_row < body_max; i++) {
        const adsb_aircraft_t *a = adsb_state_get(i);
        if (!a || !a->active) continue;

        const char *alt_col = C_TEXT;
        const char *vs_col  = C_DIM;
        char vs_buf[12]  = "  --";
        char alt_buf[12] = "      --";
        char vel_buf[12] = "    --";
        char hdg_buf[12] = "   --";

        if (a->alt_valid)
            snprintf(alt_buf, sizeof(alt_buf), "%8d", a->altitude);
        if (a->vel_valid)
            snprintf(vel_buf, sizeof(vel_buf), "%6d", a->velocity);
        if (a->hdg_valid)
            snprintf(hdg_buf, sizeof(hdg_buf), "%5d", a->heading);

        if (a->vs_valid && a->vert_rate > 200) {
            alt_col = C_GREEN;
            vs_col  = C_GREEN;
            snprintf(vs_buf, sizeof(vs_buf), "+%d", a->vert_rate);
        } else if (a->vs_valid && a->vert_rate < -200) {
            alt_col = C_AMBER;
            vs_col  = C_AMBER;
            snprintf(vs_buf, sizeof(vs_buf), "%d", a->vert_rate);
        }

        char lat_s[12] = "   ------";
        char lon_s[12] = "   ------";
        if (a->pos_valid) {
            snprintf(lat_s, sizeof(lat_s), "%+9.4f", a->lat);
            snprintf(lon_s, sizeof(lon_s), "%+9.4f", a->lon);
        }

        const char *cs = a->callsign[0] ? a->callsign : "--------";

        tui_goto(body_row, 3);
        printf(C_CYAN  "%-8lX" RESET " "
               C_GOLD  "%-9s" RESET " "
               "%s%8s" RESET " "
               C_TEXT  "%6s" RESET " "
               C_MAGENTA "%5s" RESET " "
               C_DIM   "%9s" RESET " "
               C_DIM   "%9s" RESET " "
               "%s%6s" RESET " "
               C_DIM   "%5d" RESET EL,
               (unsigned long)a->icao,
               cs, alt_col, alt_buf,
               vel_buf, hdg_buf,
               lat_s, lon_s,
               vs_col, vs_buf, a->msg_count);
        body_row++;
        shown++;
    }

    while (body_row < body_max) {
        tui_goto(body_row, 3);
        printf(C_DIM "  \xc2\xb7" RESET EL);
        body_row++;
    }

    if (shown == 0) {
        tui_goto(top + rows / 2, (cols - 30) / 2 + 3);
        printf(C_DIM "(no aircraft in range)" RESET EL);
    }
}
