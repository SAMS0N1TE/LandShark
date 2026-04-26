#include "app_registry.h"
#include "p25_state.h"
#include "sdkconfig.h"
#include <stdio.h>

#ifdef CONFIG_ENABLE_TUI

#include "tui.h"

void p25_draw_main(int top, int rows, int cols)
{
    (void)rows; (void)cols;
    tui_goto(top, 3);
    printf("P25  freq=%.4f MHz  lvl=%.2f  sync=%d  nac=%04X  tg=%d",
           s_tune_freq_hz / 1e6,
           (double)P25.iq_level,
           P25.dsd_has_sync ? 1 : 0,
           P25.dsd_nac, P25.dsd_tg);
}

void p25_on_key(tui_key_t k)
{
    (void)k;
}

#endif
