/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 * Copyright (c) 2026 dgct
 *
 * SPDX-License-Identifier: MIT
 *
 * BLE QoS channel-map filter for ZMK on the open-LL (Zephyr SW split)
 * controller.
 *
 * This is the blob-free port of the SoftDevice-Controller QoS module.
 * Instead of SDC vendor-specific HCI events (QOS_CONN_EVENT_REPORT /
 * CHANNEL_SURVEY_REPORT) and the Nordic chmap_filter library, it gets its
 * raw signal from a lightweight controller hook: lll_conn_isr_rx() calls
 * zmk_qos_crc_report(handle, chan, crc_ok) once per connection event for
 * central-role links (gated by CONFIG_BT_CTLR_QOS_CRC_HOOK).
 *
 * From those per-channel CRC statistics we maintain an EWMA-smoothed
 * error ratio per data channel, rank all 37 channels by score, and prune
 * the worst ones via HCI LE Set Host Channel Classification
 * (bt_le_set_chan_map). The host/dongle channel map (read from the
 * peripheral-role link) is folded in as a soft penalty.
 *
 * Dropped relative to the SDC version (no equivalent on open-LL or
 * blob-dependent): RF energy channel survey, adaptive energy weight, and
 * the chmap_filter WiFi-pattern library.
 *
 * Only meaningful on the split central (the side that owns the split
 * link and can call bt_le_set_chan_map).
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/buf.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/net_buf.h>

#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(zmk_ble_qos_openll, CONFIG_ZMK_BLE_QOS_OPENLL_LOG_LEVEL);

#define QOS_THREAD_PRIORITY K_PRIO_PREEMPT(K_LOWEST_APPLICATION_THREAD_PRIO)
#define QOS_INTERVAL_BASE   CONFIG_ZMK_BLE_QOS_OPENLL_INTERVAL
#define QOS_INTERVAL_FAST   CONFIG_ZMK_BLE_QOS_OPENLL_INTERVAL_FAST

/* Score weights (Q8 fixed point: 256 = 1.0) */
#define W_CRC          CONFIG_ZMK_BLE_QOS_OPENLL_SCORE_W_CRC
#define W_HOST_PENALTY CONFIG_ZMK_BLE_QOS_OPENLL_SCORE_W_HOST

/* Score threshold: channels scoring at/above this are excluded once we
 * have at least MIN_CHANNELS. Expressed in raw score units. */
#define BLOCK_THRESHOLD CONFIG_ZMK_BLE_QOS_OPENLL_SCORE_BLOCK_THRESHOLD

/* EWMA shift factor for slow decay: alpha = 1/(1<<shift). */
#define EWMA_CRC_SHIFT 3 /* alpha = 1/8, ~8-sample half-life */

#define MIN_CHANNELS CONFIG_ZMK_BLE_QOS_OPENLL_MIN_CHANNEL_COUNT

/* Map-diff hysteresis: only send LL_CHANNEL_MAP_IND when the new map
 * differs from the current by at least this many channels. Prevents
 * churn under broadband interference where the "best N" set fluctuates. */
#define MAP_DIFF_MIN 3

static K_THREAD_STACK_DEFINE(qos_stack, CONFIG_ZMK_BLE_QOS_OPENLL_STACK_SIZE);
static struct k_thread qos_thread;

static atomic_t processing;
static atomic_t crc_errors_seen;
static atomic_t burst_requested;
static atomic_t reinit_requested;
static atomic_t central_link_count;
static uint32_t current_interval_ms = QOS_INTERVAL_FAST;
static enum zmk_activity_state last_activity_state = ZMK_ACTIVITY_SLEEP;

/*
 * Per-channel CRC score state.
 *
 * crc_ratio_ewma: EWMA of CRC error ratio in Q8 (0 = perfect, 256 = 100%
 *   errors). Updated each QoS processing cycle from accumulated counts.
 *
 * crc_ok_acc / crc_err_acc: per-channel CRC accumulators filled by the
 *   controller RX ISR (zmk_qos_crc_report) and consumed by the QoS thread.
 *   Saturating uint8_t — overflow caps at 255, preserving the ratio.
 */
struct channel_score {
	int16_t crc_ratio_ewma;
	uint8_t crc_ok_acc;
	uint8_t crc_err_acc;
};

static struct channel_score ch_scores[37];

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_OPENLL_HOST_MAP_MERGE)
static struct bt_conn *host_conn;
#endif

static uint8_t last_applied_map[5];

/*
 * Controller hook — strong definition overriding the __weak no-op in
 * lll_conn.c. Called once per connection event for central-role links,
 * from the radio RX ISR. MUST stay tiny.
 *
 * Accumulates per-channel CRC ok/error counts (saturating). On the first
 * error since the last processing cycle it wakes the QoS thread so the
 * adaptive interval can drop to fast; the atomic_cas gate bounds wakeups
 * to at most one per cycle.
 */
BUILD_ASSERT(!IS_ENABLED(CONFIG_BT_CTLR_ZLI),
             "zmk_qos_crc_report() runs in the radio ISR and calls k_wakeup(): not "
             "permitted from a zero-latency interrupt");

void zmk_qos_crc_report(uint16_t handle, uint8_t chan, bool crc_ok)
{
	ARG_UNUSED(handle);

	/* Skip while the QoS thread is consuming/resetting accumulators. */
	if (atomic_get(&processing)) {
		return;
	}

	if (chan >= 37) {
		return;
	}

	struct channel_score *s = &ch_scores[chan];

	if (crc_ok) {
		if (s->crc_ok_acc < 255) {
			s->crc_ok_acc++;
		}
	} else {
		if (s->crc_err_acc < 255) {
			s->crc_err_acc++;
		}
		if (atomic_cas(&crc_errors_seen, false, true)) {
			k_wakeup(&qos_thread);
		}
	}
}

/*
 * Count set bits in a 5-byte channel map (37 data channels).
 */
static int chmap_popcount(const uint8_t map[5])
{
	int count = 0;

	for (int i = 0; i < 5; i++) {
		uint8_t b = map[i];

		while (b) {
			count++;
			b &= b - 1;
		}
	}
	return count;
}

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_OPENLL_HOST_MAP_MERGE)
/*
 * Read the channel map of the host/dongle (peripheral-role) connection —
 * the map the remote central assigned via its own AFH algorithm. Used as
 * a soft penalty: channels the host avoids are ranked lower but not hard
 * vetoed.
 */
static int read_host_chan_map(uint8_t out[5])
{
	struct bt_conn *conn = host_conn;
	struct net_buf *buf;
	struct net_buf *rsp = NULL;
	int err;

	if (!conn) {
		return -ENOTCONN;
	}

	conn = bt_conn_ref(conn);
	if (!conn) {
		return -ENOTCONN;
	}

	uint16_t handle;

	err = bt_hci_get_conn_handle(conn, &handle);
	if (err) {
		goto out;
	}

	buf = bt_hci_cmd_alloc(K_FOREVER);
	if (!buf) {
		err = -ENOMEM;
		goto out;
	}

	struct bt_hci_cp_le_read_chan_map *cp =
		net_buf_add(buf, sizeof(struct bt_hci_cp_le_read_chan_map));
	cp->handle = sys_cpu_to_le16(handle);

	err = bt_hci_cmd_send_sync(BT_HCI_OP_LE_READ_CHAN_MAP, buf, &rsp);
	if (err) {
		goto out;
	}

	struct bt_hci_rp_le_read_chan_map *rp = (void *)rsp->data;

	if (rp->status) {
		err = -EIO;
	} else {
		memcpy(out, rp->ch_map, 5);
	}

	net_buf_unref(rsp);
out:
	bt_conn_unref(conn);
	return err;
}
#endif /* CONFIG_ZMK_BLE_QOS_OPENLL_HOST_MAP_MERGE */

/*
 * Reset all channel scores to neutral state.
 * Called on init and on wake-from-sleep reinit.
 */
static void scores_reset(void)
{
	for (int ch = 0; ch < 37; ch++) {
		ch_scores[ch].crc_ratio_ewma = 0;
		ch_scores[ch].crc_ok_acc = 0;
		ch_scores[ch].crc_err_acc = 0;
	}
}

/*
 * Compute the priority score for a single channel.
 * Higher score = worse channel = lower priority to include in the map.
 *
 * Components:
 * 1. CRC error ratio (reactive: confirmed packet loss)
 * 2. Host/dongle map penalty (external intelligence from remote AFH)
 */
static int16_t compute_channel_score(int ch, const uint8_t *host_map,
				     bool host_map_valid)
{
	int32_t score = 0;
	struct channel_score *s = &ch_scores[ch];

	/* CRC component: error ratio, weighted heavily.
	 * CRC errors are confirmed packet loss — no false positives. */
	score += ((int32_t)s->crc_ratio_ewma * W_CRC) >> 8;

	/* Host/dongle map penalty: soft signal from remote central's AFH.
	 * Not a hard veto — just sorts these channels lower. If we are
	 * channel-starved they can still be included. */
	if (host_map_valid && !(host_map[ch / 8] & (1U << (ch % 8)))) {
		score += W_HOST_PENALTY;
	}

	if (score > INT16_MAX) {
		score = INT16_MAX;
	}
	return (int16_t)score;
}

/*
 * Build channel map from priority-sorted scores.
 * Includes channels in ascending score order (best first) until we hit
 * the block threshold and have at least MIN_CHANNELS. N=37 so insertion
 * sort is optimal (cache-friendly, no heap).
 */
static void build_scored_map(const uint8_t *host_map, bool host_map_valid,
			     uint8_t out_map[5])
{
	struct {
		uint8_t ch;
		int16_t score;
	} ranked[37];

	for (int ch = 0; ch < 37; ch++) {
		ranked[ch].ch = ch;
		ranked[ch].score =
			compute_channel_score(ch, host_map, host_map_valid);
	}

	/* Insertion sort ascending (best = lowest score first) */
	for (int i = 1; i < 37; i++) {
		__typeof__(ranked[0]) tmp = ranked[i];
		int j = i - 1;

		while (j >= 0 && ranked[j].score > tmp.score) {
			ranked[j + 1] = ranked[j];
			j--;
		}
		ranked[j + 1] = tmp;
	}

	/* Build map: include best channels up to threshold */
	memset(out_map, 0, 5);
	int included = 0;

	for (int i = 0; i < 37; i++) {
		if (included >= MIN_CHANNELS &&
		    ranked[i].score >= BLOCK_THRESHOLD) {
			break;
		}
		uint8_t ch = ranked[i].ch;

		out_map[ch / 8] |= (1U << (ch % 8));
		included++;
	}

	LOG_DBG("Score map: %d ch (worst_in=%d, first_out=%d)", included,
		included > 0 ? ranked[included - 1].score : 0,
		included < 37 ? ranked[included].score : 0);
}

/*
 * Process accumulated CRC data into EWMA scores.
 * Called once per QoS cycle while processing==true (safe from concurrent
 * accumulation by zmk_qos_crc_report).
 */
static void scores_process_crc(void)
{
	for (int ch = 0; ch < 37; ch++) {
		struct channel_score *s = &ch_scores[ch];
		uint8_t ok = s->crc_ok_acc;
		uint8_t err_cnt = s->crc_err_acc;

		s->crc_ok_acc = 0;
		s->crc_err_acc = 0;

		uint16_t total = (uint16_t)ok + err_cnt;

		if (total == 0) {
			/* No data this cycle — leave EWMA unchanged. */
			continue;
		}

		/* Ratio in Q8: 0 = perfect, 256 = all errors */
		int16_t ratio_q8 = ((int32_t)err_cnt << 8) / total;

		/* EWMA update: asymmetric — fast attack (alpha=1/2), slow
		 * decay (alpha=1/8). Blocks bad channels quickly, holds the
		 * block after interference clears. */
		int16_t diff = ratio_q8 - s->crc_ratio_ewma;
		int shift = (diff > 0) ? 1 : EWMA_CRC_SHIFT;

		s->crc_ratio_ewma += diff >> shift;
	}
}

/*
 * QoS thread — runs at the lowest application priority.
 *
 * Adaptive interval:
 * - Baseline: sleeps for QOS_INTERVAL_BASE
 * - On CRC errors / wake burst: drops to QOS_INTERVAL_FAST
 * - Each clean cycle: interval doubles back toward BASE
 *
 * Each cycle: process CRC accumulators into score EWMAs, read the
 * host/dongle channel map (soft penalty), build the priority-scored map,
 * and apply it if it changed by at least MAP_DIFF_MIN channels.
 */
static void qos_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		int err;

		k_sleep(K_MSEC(current_interval_ms));

		/* Nothing to do until a central (split) link exists. */
		if (atomic_get(&central_link_count) == 0) {
			continue;
		}

		/* Adapt interval: burst on CRC errors or wake-from-sleep */
		bool burst = atomic_cas(&burst_requested, true, false);
		bool errors = atomic_cas(&crc_errors_seen, true, false);

		if (burst || errors) {
			if (current_interval_ms > QOS_INTERVAL_FAST) {
				LOG_INF("QoS: %s, interval %u -> %u ms",
					errors ? "CRC errors" : "wake burst",
					current_interval_ms, QOS_INTERVAL_FAST);
			}
			current_interval_ms = QOS_INTERVAL_FAST;
		} else if (current_interval_ms < QOS_INTERVAL_BASE) {
			uint32_t next = current_interval_ms * 2;

			if (next > QOS_INTERVAL_BASE) {
				next = QOS_INTERVAL_BASE;
			}
			current_interval_ms = next;
		}

		atomic_set(&processing, true);

		/* Handle reinit request from the wake listener. Must happen
		 * inside the processing gate so zmk_qos_crc_report does not
		 * race a half-reset state. */
		if (atomic_cas(&reinit_requested, true, false)) {
			scores_reset();
			memset(last_applied_map, 0, sizeof(last_applied_map));
		}

		/* Process CRC accumulators into score EWMAs. Must complete
		 * while processing==true so the ISR hook does not race the
		 * accumulator reads/resets. */
		scores_process_crc();

		atomic_set(&processing, false);

		/* Read host/dongle channel map (soft penalty) */
		uint8_t host_map[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0x1F};
		bool host_map_valid = false;

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_OPENLL_HOST_MAP_MERGE)
		if (host_conn && !read_host_chan_map(host_map)) {
			host_map_valid = true;
		}
#endif

		/* Build priority-scored channel map */
		uint8_t final_map[5];

		build_scored_map(host_map, host_map_valid, final_map);

		/* Map-diff hysteresis — only update if the new map differs by
		 * at least MAP_DIFF_MIN channels. Prevents LL_CHANNEL_MAP_IND
		 * churn under broadband interference. */
		int diff_count = 0;

		for (int i = 0; i < 5; i++) {
			uint8_t x = final_map[i] ^ last_applied_map[i];

			while (x) {
				diff_count++;
				x &= x - 1;
			}
		}
		if (diff_count < MAP_DIFF_MIN) {
			continue;
		}

		int count = chmap_popcount(final_map);

		err = bt_le_set_chan_map(final_map);
		if (err) {
			LOG_WRN("bt_le_set_chan_map failed: %d", err);
		} else {
			memcpy(last_applied_map, final_map, 5);
			LOG_INF("Channel map updated: %d ch (interval=%u ms)",
				count, current_interval_ms);
		}
	}
}

/*
 * Connection callbacks — track the number of central-role (split) links so
 * the thread only acts when one exists, and the peripheral-role host link
 * for the soft channel-map penalty.
 */
static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	struct bt_conn_info info;

	if (err) {
		return;
	}
	if (bt_conn_get_info(conn, &info)) {
		return;
	}

	if (info.role == BT_CONN_ROLE_CENTRAL) {
		atomic_inc(&central_link_count);
		LOG_INF("QoS: central (split) link up (n=%ld)",
			atomic_get(&central_link_count));
	}
#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_OPENLL_HOST_MAP_MERGE)
	else if (info.role == BT_CONN_ROLE_PERIPHERAL) {
		if (!host_conn) {
			host_conn = bt_conn_ref(conn);
			LOG_INF("QoS: host connection tracked for score penalty");
		}
	}
#endif
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	struct bt_conn_info info;

	ARG_UNUSED(reason);

	if (!bt_conn_get_info(conn, &info) &&
	    info.role == BT_CONN_ROLE_CENTRAL) {
		atomic_dec(&central_link_count);
		LOG_INF("QoS: central (split) link down (n=%ld)",
			atomic_get(&central_link_count));
	}

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_OPENLL_HOST_MAP_MERGE)
	if (conn == host_conn) {
		LOG_INF("QoS: host connection lost");
		bt_conn_unref(host_conn);
		host_conn = NULL;
	}
#endif
}

BT_CONN_CB_DEFINE(qos_openll_conn_cb) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
};

static int qos_init(void)
{
	scores_reset();

	k_thread_create(&qos_thread, qos_stack,
			K_THREAD_STACK_SIZEOF(qos_stack), qos_thread_fn, NULL,
			NULL, NULL, QOS_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&qos_thread, "ble_qos");

	LOG_INF("BLE QoS (open-LL) priority-score system initialized");

	return 0;
}

SYS_INIT(qos_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/*
 * Activity state listener — triggers a QoS burst on wake from sleep.
 * Idle->active transitions are ignored (frequent with short idle timeout).
 * Sleep->active: the RF environment may have changed (different room).
 *
 * With REINIT_ON_WAKE: resets all scores so channels reconverge from
 * neutral within ~8 seconds (CRC EWMA half-life).
 */
static int qos_activity_listener(const zmk_event_t *eh)
{
	struct zmk_activity_state_changed *ev =
		as_zmk_activity_state_changed(eh);

	if (ev == NULL) {
		return -ENOTSUP;
	}

	enum zmk_activity_state prev = last_activity_state;

	last_activity_state = ev->state;

	if (ev->state == ZMK_ACTIVITY_ACTIVE && prev == ZMK_ACTIVITY_SLEEP) {
#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_OPENLL_REINIT_ON_WAKE)
		LOG_INF("QoS: wake from sleep, requesting reinit + burst");
		atomic_set(&reinit_requested, true);
#else
		LOG_INF("QoS: wake from sleep, requesting burst");
#endif
		atomic_set(&burst_requested, true);
		k_wakeup(&qos_thread);
	}

	return 0;
}

ZMK_LISTENER(ble_qos_openll_activity, qos_activity_listener);
ZMK_SUBSCRIPTION(ble_qos_openll_activity, zmk_activity_state_changed);
