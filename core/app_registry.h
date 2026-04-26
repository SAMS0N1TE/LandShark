/*
 * app_registry.h - multi-tool app framework.
 *
 * Apps register an app_t descriptor at boot. The framework dispatches
 * lifecycle callbacks (enter/exit/sample/key) and keeps track of which
 * app is currently active. UI rendering is opt-in: when CONFIG_ENABLE_TUI
 * is on, the TUI calls draw_main() through this registry; when off, the
 * app runs headless and emits events instead.
 */
#ifndef APP_REGISTRY_H
#define APP_REGISTRY_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PAGE_MAIN = 0,
    PAGE_WATERFALL,
    PAGE_LOG,
    PAGE_SETTINGS,
    PAGE_COUNT
} page_t;

typedef enum {
    TK_NONE  = 0,
    TK_UP    = 0x81,
    TK_DOWN  = 0x82,
    TK_LEFT  = 0x83,
    TK_RIGHT = 0x84,
    TK_ENTER = '\r',
    TK_ESC   = 0x1b,
    TK_TAB   = '\t',
    TK_BKSP  = 0x7f,
} tui_key_t;

typedef struct app_s {
    const char *name;
    uint32_t    default_freq;
    uint32_t    default_rate;
    int         default_gain;

    void (*on_enter)(void);
    void (*on_exit) (void);
    void (*on_sample)(uint8_t *iq, int len);
    void (*draw_main)(int top_row, int rows, int cols);
    void (*on_key)(tui_key_t k);
} app_t;

int          app_register(const app_t *desc);
int          app_current_index(void);
const app_t *app_current(void);
void         app_switch_to(int index);
void         app_cycle_next(void);

page_t       page_current(void);
void         page_set(page_t p);
void         page_cycle_next(void);

tui_key_t    key_feed(uint8_t byte);
tui_key_t    key_flush_timeout(void);

#ifdef __cplusplus
}
#endif

#endif
