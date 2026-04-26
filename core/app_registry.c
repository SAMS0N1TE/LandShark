/* app_registry.c - app dispatch, page state, UART key parser. */

#include "app_registry.h"
#include "event_bus.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "app";

#define MAX_APPS 4
static const app_t *s_apps[MAX_APPS];
static int          s_app_count   = 0;
static int          s_current_app = 0;
static page_t       s_page        = PAGE_MAIN;

int app_register(const app_t *desc)
{
    if (s_app_count >= MAX_APPS || !desc || !desc->name) return -1;
    s_apps[s_app_count] = desc;
    return s_app_count++;
}

int app_current_index(void)     { return s_current_app; }
const app_t *app_current(void)  { return s_app_count > 0 ? s_apps[s_current_app] : NULL; }

void app_switch_to(int idx)
{
    if (idx < 0 || idx >= s_app_count || idx == s_current_app) return;
    const app_t *old = app_current();
    if (old && old->on_exit) old->on_exit();
    int prev = s_current_app;
    s_current_app = idx;
    s_page = PAGE_MAIN;
    const app_t *nu = app_current();
    if (nu && nu->on_enter) nu->on_enter();

    event_t e = { 0 };
    e.kind = EVT_APP_SWITCHED;
    if (s_apps[prev] && s_apps[prev]->name)
        strncpy(e.u.sw.from, s_apps[prev]->name, EVT_APP_NAME_MAX);
    if (nu && nu->name)
        strncpy(e.u.sw.to, nu->name, EVT_APP_NAME_MAX);
    event_bus_publish(&e);
    ESP_LOGI(TAG, "switched to app '%s'", nu ? nu->name : "?");
}

void app_cycle_next(void)
{
    if (s_app_count <= 1) return;
    app_switch_to((s_current_app + 1) % s_app_count);
}

page_t page_current(void)    { return s_page; }
void   page_set(page_t p)    { if (p < PAGE_COUNT) s_page = p; }
void   page_cycle_next(void) { s_page = (s_page + 1) % PAGE_COUNT; }

/* UART delivers arrow keys as 3-byte sequences (ESC '[' A/B/C/D). The
 * polling loop calls uart_read_bytes with timeout 0, so we feed bytes
 * through this state machine one at a time. key_flush_timeout() recovers
 * from a stalled mid-sequence (bare ESC or interrupted CSI). */
typedef enum { KP_IDLE = 0, KP_ESC, KP_CSI } kp_state_t;
static kp_state_t s_kp_state = KP_IDLE;
static int        s_kp_ticks = 0;

tui_key_t key_feed(uint8_t b)
{
    s_kp_ticks = 0;
    switch (s_kp_state) {
    case KP_IDLE:
        if (b == 0x1b) { s_kp_state = KP_ESC; return TK_NONE; }
        if (b == 0x08 || b == 0x7f) return TK_BKSP;
        if (b == '\r' || b == '\n') return TK_ENTER;
        return (tui_key_t)b;
    case KP_ESC:
        if (b == '[') { s_kp_state = KP_CSI; return TK_NONE; }
        s_kp_state = KP_IDLE;
        return TK_ESC;
    case KP_CSI:
        s_kp_state = KP_IDLE;
        switch (b) {
            case 'A': return TK_UP;
            case 'B': return TK_DOWN;
            case 'C': return TK_RIGHT;
            case 'D': return TK_LEFT;
            default:  return TK_NONE;
        }
    }
    return TK_NONE;
}

tui_key_t key_flush_timeout(void)
{
    if (s_kp_state == KP_IDLE) return TK_NONE;
    if (++s_kp_ticks < 3) return TK_NONE;
    s_kp_state = KP_IDLE;
    s_kp_ticks = 0;
    return TK_ESC;
}
