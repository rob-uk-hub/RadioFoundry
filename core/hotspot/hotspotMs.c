// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
// Hotspot MS mode: a mobile station on a repeater, both timeslots (see hotspotMs.h). Without BS sync, BS_Dwn_Act
// wakes the repeater; no host frame is keyed until it answers.
#include "config.h"
#if CONFIG_HOTSPOT_MS	// the whole file

#include <string.h>
#include "hotspot/hotspotMs.h"
#include "hotspot/hotspot.h"
#include "hotspot/mmdvm.h"
#include "hotspot/hotspotSlot.h"
#include "hotspot/dmrDataFrame.h"
#include "dmr/dmrData.h"
#include "dmr/fec/dmrFec.h"
#include "dmr/call/dmr_lc.h"

#define MS_LC_BYTES	HOTSPOT_MS_LC_BYTES
#define MS_GAP_MS	200U	// no host frame for this long: unkey once the queues drain
#define MS_SYNC_MS	300U	// a BS-synced burst this recently: TX may key (within the modem's slot timing)
#define MS_WAKE_ATTEMPTS	3U	// BS_Dwn_Act tries before the frames wait for a new call
#define MS_WAKE_LISTEN_MS	600U	// after each BS_Dwn_Act, before the next
#define MS_DATA_HOLD_MS	180U	// collect a host data list (header and blocks) before keying
#define MS_DATA_GAP_MS	180U	// no more data frames for this long: end the data transmission
#define MS_SPACE_MARGIN	2
#define MS_SW_DEPTH	DMR_DATA_MAX_BURSTS	// one host data list can fill it
_Static_assert(MS_SW_DEPTH < 128, "the 8-bit sequence wraps only beyond the queue");

typedef struct {
	dmr_emb_rx_t emb_rx;
	uint8_t emb_lc[MS_LC_BYTES];
	bool emb_valid;
	uint8_t net_index;	// the last voice position the host sent
	bool net_term;		// the host's last frame was a terminator: its repeats are dropped
} ms_slot_t;

static struct {
	bool enabled;
	uint8_t cc;		// the color code the state belongs to
	bool keyed;
	bool end_req;
	bool bs_seen;		// a BS-synced burst has been heard in this mode
	uint32_t bs_ms;		// ... and when
	bool waking;		// a BS_Dwn_Act is on air / queued
	uint32_t wake_ms;	// when the last BS_Dwn_Act went out
	uint8_t wake_tries;
	uint32_t last_data_ms[2];	// the last data frame queued on each slot
	bool data_open;		// a data transmission is keyed and still being fed
	uint8_t data_slot;	// 0 TS1, 1 TS2
	uint32_t last_frame_ms;
	hotspotSlot_t rx[2];
	ms_slot_t tx[2];
	hotspotMsItem_t *sw;		// the pool hotspot.c lends (hotspot_ms_storage())
	uint8_t sw_next[MS_SW_DEPTH];	// each slot's chain, then the free list
	uint8_t sw_head[2], sw_tail[2], sw_count[2];
	uint8_t sw_free_head, sw_free_count;
	uint8_t sw_seq;
} ms;

static const hal_dmr_modem_t *modem(void)
{
	return hotspot_modem();
}

static bool supported(void)
{
	return hotspot_ms_supported();
}

static bool use_hw(void)
{
	const hal_dmr_modem_t *m = modem();

	return (m != NULL) && (m->ms_begin != NULL) && (m->ms_queue != NULL) && (m->ms_space != NULL) &&
	       (m->ms_end != NULL) && (m->ms_abort != NULL);
}

static bool bs_synced(uint32_t now_ms)
{
	return ms.bs_seen && ((uint32_t)(now_ms - ms.bs_ms) <= MS_SYNC_MS);
}

static bool is_data(uint8_t kind)
{
	return (kind == DMR_BURST_CSBK) || (kind == DMR_BURST_MBC_HEADER) || (kind == DMR_BURST_MBC_CONT) ||
	       (kind == DMR_BURST_DATA_HEADER) || (kind == DMR_BURST_RATE_12_DATA) ||
	       (kind == DMR_BURST_RATE_34_DATA) || (kind == DMR_BURST_RATE_1_DATA);
}

// Both slots keep their own order in one pool, so a data list on one slot stays one key-up while the other slot
// waits. The pool is shared: a host that fills both slots gets NAK 5 at 64 frames in all.
static uint8_t sw_space(void)
{
	return ms.sw_free_count;
}

static bool sw_push(int k, const dmr_tx_burst_t *b, const uint8_t *lc)
{
	hotspotMsItem_t *it;
	uint8_t i;

	if ((ms.sw == NULL) || (ms.sw_free_count == 0)) {
		return false;
	}
	i = ms.sw_free_head;
	ms.sw_free_head = ms.sw_next[i];
	ms.sw_free_count--;
	it = &ms.sw[i];
	it->b = *b;
	it->has_lc = (lc != NULL);
	if (lc != NULL) {
		memcpy(it->lc, lc, MS_LC_BYTES);
	}
	it->seq = ms.sw_seq++;
	ms.sw_next[i] = 0xFF;
	if (ms.sw_tail[k] == 0xFF) {
		ms.sw_head[k] = i;
	} else {
		ms.sw_next[ms.sw_tail[k]] = i;
	}
	ms.sw_tail[k] = i;
	ms.sw_count[k]++;
	return true;
}

static bool sw_pick(int *k_out)
{
	bool found = false;
	int k_best = 0;
	uint8_t seq_best = 0;

	for (int k = 0; k < 2; k++) {
		if (ms.sw_count[k] == 0) {
			continue;
		}
		uint8_t seq = ms.sw[ms.sw_head[k]].seq;
		if (!found || ((int8_t)(seq - seq_best) < 0)) {
			found = true;
			k_best = k;
			seq_best = seq;
		}
	}
	*k_out = k_best;
	return found;
}

static void sw_pop(int k)
{
	uint8_t i;

	if (ms.sw_head[k] == 0xFF) {
		return;
	}
	i = ms.sw_head[k];
	ms.sw_head[k] = ms.sw_next[i];
	if (ms.sw_head[k] == 0xFF) {
		ms.sw_tail[k] = 0xFF;
	}
	ms.sw_next[i] = ms.sw_free_head;
	ms.sw_free_head = i;
	ms.sw_free_count++;
	ms.sw_count[k]--;
}

static void sw_clear(void)
{
	for (int i = 0; i < MS_SW_DEPTH; i++) {
		ms.sw_next[i] = (uint8_t)((i + 1 < MS_SW_DEPTH) ? (i + 1) : 0xFF);
	}
	ms.sw_free_head = 0;
	ms.sw_free_count = MS_SW_DEPTH;
	for (int k = 0; k < 2; k++) {
		ms.sw_head[k] = ms.sw_tail[k] = 0xFF;
		ms.sw_count[k] = 0;
	}
	ms.sw_seq = 0;
}

// The standard TX path can send one burst per key-up. Start the oldest queued burst; the rest follow on TX_DONE.
// Nothing keys before a BS-synced burst says the repeater is there (an MS on a repeater aligns to it).
// Wake the repeater with BS_Dwn_Act on our own timing: there is no BS sync to align to yet. The host frames wait
// until it answers.
static void wake_poll(uint32_t now_ms)
{
	const hal_dmr_modem_t *m = modem();
	uint8_t csbk[12];
	dmr_tx_burst_t b;
	uint32_t id;
	int k;

	if (!ms.enabled || use_hw() || (m == NULL) || (m->tx_begin == NULL)) {
		return;
	}
	if (bs_synced(now_ms)) {
		ms.wake_tries = 0;	// a data stream in progress still clears the wake attempts
		return;
	}
	if (ms.waking || ms.keyed) {
		return;
	}
	if ((ms.sw_count[0] + ms.sw_count[1]) == 0) {
		ms.wake_tries = 0;
		return;
	}
	if (ms.wake_tries >= MS_WAKE_ATTEMPTS) {
		return;
	}
	if ((ms.wake_tries > 0) && ((uint32_t)(now_ms - ms.wake_ms) < MS_WAKE_LISTEN_MS)) {
		return;
	}
	id = hotspot_dmr_id();
	if (id == 0) {
		return;
	}
	if (!sw_pick(&k)) {
		return;
	}
	dmr_csbk_bs_activation(id, csbk);
	memset(&b, 0, sizeof(b));
	b.kind = DMR_BURST_CSBK;
	b.host_crc = true;
	b.len = 12;
	memcpy(b.payload, csbk, sizeof(csbk));
	if (!m->tx_begin(DMR_TIMING_OWN, (int8_t)(k + 1), DMR_ACCESS_IMPOLITE)) {
		return;
	}
	if (!m->tx_queue(&b)) {
		m->tx_abort();
		return;
	}
	m->tx_end();
	ms.keyed = true;
	ms.waking = true;
	ms.wake_ms = now_ms;
	ms.wake_tries++;
}

// Keep a data list's transmission open and feed the driver as its queue drains: a long SMS is one key-up
static void data_feed(uint32_t now_ms)
{
	const hal_dmr_modem_t *m = modem();
	uint8_t k = ms.data_slot;

	if (!ms.data_open || !ms.keyed || !ms.enabled || (m == NULL)) {
		return;
	}
	while ((ms.sw_count[k] > 0) && (m->tx_queue_space() > 0) &&
	       is_data(ms.sw[ms.sw_head[k]].b.kind)) {
		hotspotMsItem_t *it = &ms.sw[ms.sw_head[k]];

		if (!m->tx_queue(&it->b)) {
			return;
		}
		sw_pop(k);
	}
	if ((ms.sw_count[k] > 0) && !is_data(ms.sw[ms.sw_head[k]].b.kind)) {
		m->tx_end();		// a terminator or a voice frame follows: end the data transmission first
		ms.data_open = false;
		ms.end_req = true;
		ms.wake_tries = 0;
	} else if ((ms.sw_count[k] == 0) && ((uint32_t)(now_ms - ms.last_data_ms[k]) >= MS_DATA_GAP_MS)) {
		m->tx_end();
		ms.data_open = false;
		ms.end_req = true;
		ms.wake_tries = 0;	// the next message wakes from a fresh count
	}
}

static void sw_kick(uint32_t now_ms)
{
	const hal_dmr_modem_t *m = modem();
	int k;
	hotspotMsItem_t *it;

	if (ms.keyed || !ms.enabled || use_hw() || (m == NULL) || !bs_synced(now_ms)) {
		return;
	}
	if (!sw_pick(&k)) {
		return;
	}
	it = &ms.sw[ms.sw_head[k]];
	if (is_data(it->b.kind)) {
		if ((uint32_t)(now_ms - ms.last_data_ms[k]) < MS_DATA_HOLD_MS) {
			return;	// the rest of the host's data list may still follow
		}
		if (!m->tx_begin(DMR_TIMING_ALIGNED, (int8_t)(k + 1), DMR_ACCESS_IMPOLITE)) {
			return;
		}
		ms.keyed = true;
		ms.end_req = false;
		ms.data_open = true;
		ms.data_slot = (uint8_t)k;
		data_feed(now_ms);
		return;
	}
	if (!m->tx_begin(DMR_TIMING_ALIGNED, (int8_t)(k + 1), DMR_ACCESS_IMPOLITE)) {
		return;
	}
	if (it->has_lc && (m->set_embedded_lc != NULL)) {
		m->set_embedded_lc(it->lc);
	}
	if (!m->tx_queue(&it->b)) {
		m->tx_abort();
		return;
	}
	m->tx_end();
	ms.keyed = true;
	ms.end_req = false;
	sw_pop(k);
}

static void rx_end(void)
{
	for (int k = 0; k < 2; k++) {
		if (hotspotUplinkCallOpen(&ms.rx[k].up)) {
			hotspotSlotLost(&ms.rx[k], k);
		}
		hotspotSlotReset(&ms.rx[k]);
	}
}

static void tx_end(void)
{
	const hal_dmr_modem_t *m = modem();

	if (ms.keyed && (m != NULL)) {
		if (m->ms_abort != NULL) {
			m->ms_abort();
		} else if (m->tx_abort != NULL) {
			m->tx_abort();
		}
	}
	sw_clear();
	ms.keyed = false;
	ms.end_req = false;
	ms.waking = false;
	ms.wake_tries = 0;
	ms.data_open = false;
	for (int k = 0; k < 2; k++) {
		dmr_emb_rx_reset(&ms.tx[k].emb_rx);
		ms.tx[k].emb_valid = false;
		ms.tx[k].net_term = false;
	}
}

static void stop_state(void)
{
	rx_end();
	tx_end();
	hotspot_alt_data_reset();
	ms.bs_seen = false;
	ms.waking = false;
	ms.wake_tries = 0;
	ms.data_open = false;
	memset(&ms.rx, 0, sizeof(ms.rx));
	memset(&ms.tx, 0, sizeof(ms.tx));
}

// Follow SET_CONFIG: mode and color code changes drop what belongs to the last one
static void follow_config(void)
{
	bool want = hotspot_ms_mode() && supported();
	uint8_t cc = hotspot_color_code();

	if (!want) {
		if (ms.enabled) {
			stop_state();
		}
		ms.enabled = false;
		return;
	}
	if (!ms.enabled || (cc != ms.cc)) {
		if (ms.enabled) {
			stop_state();
		}
		ms.cc = cc;
		ms.enabled = true;
	}
}

static bool alt_active(void)
{
	follow_config();
	return ms.enabled;
}

static void alt_poll(uint32_t now_ms)
{
	follow_config();
	if (!ms.enabled) {
		return;
	}
	for (int k = 0; k < 2; k++) {
		hotspotSlotPoll(&ms.rx[k], k, now_ms);
	}
	if (!use_hw()) {
		data_feed(now_ms);
		wake_poll(now_ms);
		sw_kick(now_ms);
		return;
	}
	if (ms.keyed && !ms.end_req && ((uint32_t)(now_ms - ms.last_frame_ms) >= MS_GAP_MS)) {
		const hal_dmr_modem_t *m = modem();

		if ((m != NULL) && (m->ms_end != NULL)) {
			m->ms_end();
			ms.end_req = true;
		}
	}
}

static void alt_event(uint8_t kind, const dmr_burst_t *burst, uint32_t now_ms)
{
	follow_config();
	if (!ms.enabled) {
		return;
	}
	switch (kind) {
	case DMR_EV_BURST_RX:
		if (burst != NULL) {
			int k = (burst->ts == 1) ? 0 : 1;	// no slot number: slot 2, as the simplex engine

			if (burst->sync == DMR_SYNC_BS) {
				ms.bs_seen = true;
				ms.bs_ms = now_ms;
			}
			hotspotSlotBurst(&ms.rx[k], k, burst, now_ms);
		}
		break;
	case DMR_EV_TX_DONE:
		ms.keyed = false;
		ms.end_req = false;
		ms.waking = false;
		ms.data_open = false;
		wake_poll(now_ms);
		sw_kick(now_ms);
		break;
	case DMR_EV_TX_REJECTED:
		ms.keyed = false;
		ms.end_req = false;
		ms.waking = false;
		ms.data_open = false;
		wake_poll(now_ms);
		sw_kick(now_ms);
		break;
	case DMR_EV_SYNC_LOST:
		ms.bs_seen = false;
		break;
	default:
		break;
	}
}

static void frame_emb(const uint8_t *frame, uint8_t *emb_hi, uint8_t frag[4], uint8_t *emb_lo)
{
	uint8_t mid[6];

	for (int k = 0; k < 6; k++) {
		mid[k] = (uint8_t)((frame[13 + k] << 4) | (frame[14 + k] >> 4));
	}
	*emb_hi = mid[0];
	memcpy(frag, mid + 1, 4);
	*emb_lo = mid[5];
}

static uint8_t net_burst(int8_t ts, uint8_t control, const uint8_t *frame, dmr_tx_burst_t *burst,
			 const uint8_t **lc_out)
{
	int k = ts - 1;
	uint8_t c = hotspot_frame_control(control, frame);
	dmr_tx_burst_t b;
	const uint8_t *lc = NULL;

	memset(&b, 0, sizeof(b));
	if (c & MMDVM_DMR_SYNC_DATA) {
		uint8_t dt = c & 0x0F;

		if (dt == DMR_TYPE_IDLE) {
			return 0xFF;	// the host has nothing for this slot
		}
		if ((dt == DMR_TYPE_TERMINATOR_LC) && ms.tx[k].net_term) {
			// The host repeats the terminator (normally 3 times) so it survives a lost burst; here the
			// radio's own timing repeats bursts, so drop the repeats and send it once.
			return 0xFF;
		}
		b.kind = dt;
		b.host_crc = true;	// the host's CRC goes out as sent (CRC_MCU_Control)
		if ((dt == DMR_TYPE_VOICE_LC_HEADER) || (dt == DMR_TYPE_TERMINATOR_LC) || (dt == DMR_TYPE_PI_HEADER)) {
			dmrFecBptcDecode(frame, b.payload);
			b.len = 12;
			if (dt == DMR_TYPE_VOICE_LC_HEADER) {
				memcpy(ms.tx[k].emb_lc, b.payload, MS_LC_BYTES);
				ms.tx[k].emb_valid = true;
				dmr_emb_rx_reset(&ms.tx[k].emb_rx);
			}
		} else {
			dmrBurst_t d;

			if (!dmrDataFrameToBurst(dt, frame, &d) || (d.length > DMR_PAYLOAD_MAX)) {
				return MMDVM_NAK_INVALID;
			}
			b.len = d.length;
			memcpy(b.payload, d.payload, d.length);
		}
	} else {
		uint8_t index;

		if (c & MMDVM_DMR_SYNC_AUDIO) {
			index = 0;
		} else if (((c & 0x0F) >= 1) && ((c & 0x0F) <= 5)) {
			index = c & 0x0F;
		} else {
			index = (uint8_t)((ms.tx[k].net_index + 1) % 6);	// control byte 0: B-F follow in order
		}
		b.kind = DMR_BURST_VOICE;
		b.voice_index = index;
		b.len = DMR_VOICE_BYTES;
		hotspot_frame_voice(frame, b.payload);
		ms.tx[k].net_index = index;
		if ((index >= 1) && (index <= 4)) {
			// The host's embedded LC (talker alias and GPS pass through): used from the next superframe
			uint8_t hi, lo, frag[4], cc, lcss, out[MS_LC_BYTES];
			bool pi;

			frame_emb(frame, &hi, frag, &lo);
			uint16_t emb = (uint16_t)((hi << 8) | lo);

			if ((dmrFecEmbDecode(emb, &cc, &pi, &lcss) >= 0) &&
			    dmr_emb_rx_push(&ms.tx[k].emb_rx, frag, lcss, out)) {
				memcpy(ms.tx[k].emb_lc, out, MS_LC_BYTES);
				ms.tx[k].emb_valid = true;
			}
		} else if ((index == 0) && ms.tx[k].emb_valid) {
			lc = ms.tx[k].emb_lc;
		}
	}
	ms.tx[k].net_term = (b.kind == DMR_BURST_TERMINATOR_LC);
	*burst = b;
	*lc_out = lc;
	return 0;
}

static uint8_t alt_net_frame(uint8_t command, uint8_t control, const uint8_t *frame, uint32_t now_ms)
{
	const hal_dmr_modem_t *m = modem();
	int8_t ts = (command == MMDVM_DMR_DATA1) ? 1 : 2;
	dmr_tx_burst_t b;
	const uint8_t *lc;
	uint8_t r;

	follow_config();
	if (!ms.enabled || (m == NULL) || (m->tx_begin == NULL)) {
		return MMDVM_NAK_WRONG_MODE;
	}
	r = net_burst(ts, control, frame, &b, &lc);
	if (r == 0xFF) {
		return 0;
	}
	if (r != 0) {
		return r;
	}
	if (use_hw()) {
		// ms_begin owns the BS-sync wait, then keys (hal_dmr_modem.h); the fallback below is held here
		if (m->ms_space(ts) <= MS_SPACE_MARGIN) {
			return MMDVM_NAK_FULL;
		}
		if (!ms.keyed) {
			if (!m->ms_begin(DMR_ACCESS_IMPOLITE)) {
				return MMDVM_NAK_WRONG_MODE;
			}
			ms.keyed = true;
		}
		ms.end_req = false;
		if (!m->ms_queue(ts, &b, lc)) {
			if (m->ms_abort != NULL) {
				m->ms_abort();
			}
			ms.keyed = false;
			return MMDVM_NAK_FULL;
		}
	} else {
		bool first = (ms.sw_count[0] + ms.sw_count[1]) == 0;

		if (sw_space() <= MS_SPACE_MARGIN) {
			return MMDVM_NAK_FULL;
		}
		if (!sw_push((int8_t)(ts - 1), &b, lc)) {
			return MMDVM_NAK_FULL;
		}
		if (first || (b.kind == DMR_BURST_DATA_HEADER)) {
			ms.wake_tries = 0;	// a new call (or a new data list) wakes again
		}
		if (is_data(b.kind)) {
			ms.last_data_ms[ts - 1] = now_ms;
		}
		data_feed(now_ms);
		wake_poll(now_ms);
		sw_kick(now_ms);
	}
	ms.last_frame_ms = now_ms;
	return 0;
}

static void alt_net_abort(void)
{
	tx_end();
}

static bool alt_transmitting(void)
{
	const hal_dmr_modem_t *m = modem();

	return ms.keyed || ((m != NULL) &&
		(((m->ms_keyed != NULL) && m->ms_keyed()) || ((m->is_transmitting != NULL) && m->is_transmitting())));
}

static void alt_stopped(void)
{
	stop_state();
	ms.enabled = false;
}

static uint8_t alt_space(int8_t ts)
{
	const hal_dmr_modem_t *m = modem();
	int s = use_hw() ? m->ms_space(ts) : sw_space();

	s -= MS_SPACE_MARGIN;
	return (uint8_t)((s < 0) ? 0 : ((s > 255) ? 255 : s));
}

static const hotspot_alt_t alt = {
	.active = alt_active, .poll = alt_poll, .event = alt_event, .net_frame = alt_net_frame,
	.net_abort = alt_net_abort, .transmitting = alt_transmitting, .stopped = alt_stopped,
	.space = alt_space,
};

void hotspot_ms_init(hotspotMsItem_t *storage)
{
	memset(&ms, 0, sizeof(ms));
	ms.sw = storage;
	sw_clear();
	for (int k = 0; k < 2; k++) {
		hotspotSlotReset(&ms.rx[k]);
		dmr_emb_rx_reset(&ms.tx[k].emb_rx);
	}
	hotspot_set_alt_ms(&alt);
}

#endif
