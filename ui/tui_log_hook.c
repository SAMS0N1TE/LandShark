/* tui_log_hook.c - ESP_LOG capture into the TUI log page. */

#include "tui_log_hook.h"
#include "tui.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

static int tui_vprintf_hook(const char *fmt, va_list args)
{
    char buf[128];
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    if (n <= 0) return n;
    if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;

    /* Strip leading ANSI colour escape "\033[0;xx;xxm" emitted by ESP_LOG
     * when CONFIG_LOG_COLORS=y. */
    const char *p = buf;
    if (*p == '\033') {
        while (*p && *p != 'm') p++;
        if (*p == 'm') p++;
    }

    int color = 0;
    if      (p[0] == 'E') color = 4;
    else if (p[0] == 'W') color = 2;
    else if (p[0] == 'I') color = 1;
    else if (p[0] == 'D') color = 5;
    else if (p[0] == 'V') color = 5;

    int len = (int)strlen(p);
    while (len > 0) {
        char c = p[len - 1];
        if (c == '\n' || c == '\r') { len--; continue; }
        break;
    }
    if (len >= 4 && p[len - 4] == '\033') len -= 4;

    char clean[96];
    if (len >= (int)sizeof(clean)) len = sizeof(clean) - 1;
    memcpy(clean, p, len);
    clean[len] = 0;

    if (len > 0) tui_log(color, "%s", clean);
    return n;
}

void tui_install_log_hook(void)
{
    typedef int (*vprintf_like_t)(const char *, va_list);
    extern vprintf_like_t esp_log_set_vprintf(vprintf_like_t func);
    esp_log_set_vprintf(tui_vprintf_hook);
}
