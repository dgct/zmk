/*
 * Copyright (c) 2026 Dan Tree
 * SPDX-License-Identifier: MIT
 *
 * Cross-hand event consolidation for split keyboards.
 *
 * Transport delay can cause events from the remote half to arrive at the
 * central after locally-generated events that physically happened later.
 * This listener sits BEFORE hold-tap in the event subscription chain and
 * briefly buffers local key-presses when the split transport is active,
 * flushing in corrected timestamp order.
 *
 * Design properties:
 *   - Zero overhead when the transport is idle (single-hand typing)
 *   - Early flush: the instant a remote event arrives, all buffered
 *     events are sorted and dispatched — no waiting for the full window
 *   - Only local PRESSES are buffered; releases and remote events flow
 *     through immediately (unless a buffer is already in flight)
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define WINDOW_US  CONFIG_ZMK_SPLIT_CONSOLIDATION_US
#define ACTIVE_MS  CONFIG_ZMK_SPLIT_CONSOLIDATION_ACTIVE_MS
#define MAX_PENDING 8

/*
 * Weak symbol — overridden by the split transport (e.g. ESB central.c)
 * with the k_uptime_get() of the most recent key-delta packet.  Default
 * INT64_MIN means "no transport activity ever" → consolidation always
 * passes events through with zero latency.
 */
int64_t __weak esb_last_key_delta_arrival_ms = INT64_MIN;

struct pending_event {
    struct zmk_position_state_changed_event ev;
};

static struct pending_event pending[MAX_PENDING];
static int pending_count;

static void flush_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(flush_work, flush_work_handler);

/* Forward-declare for ZMK_EVENT_RAISE_AFTER in flush_pending(). */
extern const struct zmk_listener zmk_listener_consolidation;

static bool transport_recently_active(void) {
    int64_t age = k_uptime_get() - esb_last_key_delta_arrival_ms;
    return age >= 0 && age < ACTIVE_MS;
}

static void flush_pending(void) {
    if (pending_count == 0) {
        return;
    }

    /* Insertion sort by timestamp (N ≤ 8). */
    for (int i = 1; i < pending_count; i++) {
        struct pending_event tmp = pending[i];
        int j = i - 1;
        while (j >= 0 &&
               pending[j].ev.data.timestamp > tmp.ev.data.timestamp) {
            pending[j + 1] = pending[j];
            j--;
        }
        pending[j + 1] = tmp;
    }

    int n = pending_count;
    pending_count = 0;

    for (int i = 0; i < n; i++) {
        LOG_DBG("consolidation flush pos=%u %s ts=%lld",
                pending[i].ev.data.position,
                pending[i].ev.data.state ? "dn" : "up",
                pending[i].ev.data.timestamp);
        ZMK_EVENT_RAISE_AFTER(pending[i].ev, consolidation);
    }
}

static void flush_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    flush_pending();
}

static int consolidation_listener(const zmk_event_t *eh) {
    struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (!ev) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    bool is_local =
        (ev->source == ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL);

    /* ── Fast path: nothing buffered ── */
    if (pending_count == 0) {
        /*
         * Only buffer a LOCAL PRESS when the transport is active.
         * Everything else passes through with zero latency:
         *   - Remote events: already delayed by transport, no benefit
         *   - Local releases: the matching press already went through
         *   - Local presses with no transport activity: no cross-hand
         *     race possible
         */
        if (!is_local || !ev->state || !transport_recently_active()) {
            return ZMK_EV_EVENT_BUBBLE;
        }

        pending[0].ev = copy_raised_zmk_position_state_changed(ev);
        pending_count = 1;
        k_work_schedule(&flush_work, K_USEC(WINDOW_US));
        LOG_DBG("consolidation buffer local press pos=%u", ev->position);
        return ZMK_EV_EVENT_CAPTURED;
    }

    /* ── Buffer non-empty: add this event to maintain ordering ── */
    if (pending_count >= MAX_PENDING) {
        LOG_WRN("consolidation buffer full, force flush");
        k_work_cancel_delayable(&flush_work);
        flush_pending();
        return ZMK_EV_EVENT_BUBBLE;
    }

    pending[pending_count].ev =
        copy_raised_zmk_position_state_changed(ev);
    pending_count++;

    /*
     * Remote event arrived while locals are buffered — this is the event
     * we were waiting for.  Flush immediately: sort everything by
     * timestamp and dispatch.  The remote HRM (with an earlier corrected
     * timestamp) will precede the buffered local press, so hold-tap sees
     * them in the correct physical order.
     */
    if (!is_local) {
        LOG_DBG("consolidation early flush on remote (%d events)",
                pending_count);
        k_work_cancel_delayable(&flush_work);
        flush_pending();
    }

    return ZMK_EV_EVENT_CAPTURED;
}

ZMK_LISTENER(consolidation, consolidation_listener);
ZMK_SUBSCRIPTION(consolidation, zmk_position_state_changed);
