/* event_stream.c - JSONL formatter, prints one record-separated line per event. */

#include "event_stream.h"
#include "event_bus.h"
#include <stdio.h>
#include <string.h>

#define RS "\x1e"  /* ASCII record separator */

static void emit_contact(const event_t *e)
{
    const evt_contact_t *c = &e->u.contact;
    printf(RS "{\"t\":%lld,\"k\":\"%s\",\"app\":\"%s\","
           "\"icao\":\"%06lX\",\"cs\":\"%s\","
           "\"alt\":%d,\"vel\":%d,\"hdg\":%d,\"vs\":%d,"
           "\"lat\":%.5f,\"lon\":%.5f,\"pos\":%s,\"shaky\":%s}\n",
           (long long)e->ts_us, evt_kind_name(e->kind), e->app,
           (unsigned long)c->icao, c->callsign,
           c->altitude, c->velocity, c->heading, c->vert_rate,
           c->lat, c->lon,
           c->pos_valid ? "true" : "false",
           c->crc_shaky ? "true" : "false");
}

static void emit_heartbeat(const event_t *e)
{
    const evt_heartbeat_t *h = &e->u.hb;
    printf(RS "{\"t\":%lld,\"k\":\"%s\",\"app\":\"%s\","
           "\"bps\":%lu,\"msgs\":%d,\"mps\":%d,"
           "\"crc_good\":%d,\"crc_err\":%d,\"ac\":%d,"
           "\"mag_avg\":%d,\"mag_peak\":%d}\n",
           (long long)e->ts_us, evt_kind_name(e->kind), e->app,
           (unsigned long)h->bytes_per_sec, h->msgs_total, h->msgs_per_sec,
           h->crc_good, h->crc_err, h->active_count,
           h->mag_avg, h->mag_peak);
}

static void emit_app_switch(const event_t *e)
{
    printf(RS "{\"t\":%lld,\"k\":\"%s\",\"from\":\"%s\",\"to\":\"%s\"}\n",
           (long long)e->ts_us, evt_kind_name(e->kind),
           e->u.sw.from, e->u.sw.to);
}

static void emit_simple(const event_t *e)
{
    printf(RS "{\"t\":%lld,\"k\":\"%s\",\"app\":\"%s\"}\n",
           (long long)e->ts_us, evt_kind_name(e->kind), e->app);
}

static void on_event(const event_t *e, void *user)
{
    (void)user;
    if (!e) return;
    switch (e->kind) {
    case EVT_CONTACT_NEW:
    case EVT_CONTACT_CONFIRMED:
    case EVT_CONTACT_LOST:
    case EVT_CONTACT_POSITION:
    case EVT_CONTACT_ALTITUDE:
    case EVT_CONTACT_VELOCITY:
    case EVT_CONTACT_IDENT:
        emit_contact(e);
        break;
    case EVT_HEARTBEAT:
        emit_heartbeat(e);
        break;
    case EVT_APP_SWITCHED:
        emit_app_switch(e);
        break;
    case EVT_BOOT:
    case EVT_SHUTDOWN:
    case EVT_DEVICE_ATTACHED:
    case EVT_DEVICE_DETACHED:
    case EVT_TUNER_LOCKED:
        emit_simple(e);
        break;
    case EVT_LOG:
    case EVT_DECODE_ERROR:
    default:
        break;
    }
    fflush(stdout);
}

void event_stream_init(void)
{
    event_bus_subscribe(on_event, NULL);
}
