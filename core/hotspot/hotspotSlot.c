// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
// One received timeslot for the host (see hotspotSlot.h)
#include <string.h>
#include "hotspot/hotspotSlot.h"
#include "hotspot/hotspot.h"
#include "hotspot/mmdvm.h"
#include "hotspot/dmrDataFrame.h"
#include "dmr/dmrData.h"
#include "dmr/fec/dmrFec.h"
#include "dmr/call/dmr_lc.h"

#define LC_LEN	9

static uint8_t up_cmd(int k, bool lost)
{
	if (lost) {
		return (k == 0) ? MMDVM_DMR_LOST1 : MMDVM_DMR_LOST2;
	}
	return (k == 0) ? MMDVM_DMR_DATA1 : MMDVM_DMR_DATA2;
}

void hotspotSlotLost(hotspotSlot_t *r, int k)
{
	hotspot_up_lost(up_cmd(k, true));
	r->alias_pending = false;
}

void hotspotSlotReset(hotspotSlot_t *r)
{
	memset(r, 0, sizeof(*r));
	hotspotUplinkReset(&r->up);
}

static const uint8_t MS_VOICE_SYNC[6] = { 0x7F, 0x7D, 0x5D, 0xD5, 0x7D, 0xFD };

// A voice burst up, as the simplex uplink frames it (mmdvm-protocol.md 2.8) with this slot's own embedded LC
static void rx_voice(int k, hotspotSlot_t *r, const uint8_t *voice, uint8_t index, int16_t rssi_dbm, bool heard)
{
	uint8_t frame[MMDVM_DMR_FRAME_LEN];
	uint8_t middle[6] = { 0, 0, 0, 0, 0, 0 };
	uint8_t payload[1 + MMDVM_DMR_FRAME_LEN + 2];
	uint8_t len = 1 + MMDVM_DMR_FRAME_LEN;
	uint8_t raw;

	if (index == 0) {
		dmr_emb_lc_encode(r->alias_pending ? r->alias_lc : hotspotUplinkLc(&r->up), r->sf_frag);
		r->alias_pending = false;
		memcpy(middle, MS_VOICE_SYNC, 6);
	} else {
		uint16_t emb = dmrFecEmbEncode(hotspot_color_code(), false, dmr_emb_lcss(index));

		middle[0] = (uint8_t)(emb >> 8);
		if (index <= 4) {
			memcpy(middle + 1, r->sf_frag[index - 1], 4);
		}
		middle[5] = (uint8_t)emb;
	}
	memcpy(frame, voice, 13);
	frame[13] = (uint8_t)((voice[13] & 0xF0) | (middle[0] >> 4));
	for (int i = 0; i < 5; i++) {
		frame[14 + i] = (uint8_t)((middle[i] << 4) | (middle[i + 1] >> 4));
	}
	frame[19] = (uint8_t)((middle[5] << 4) | (voice[13] & 0x0F));
	memcpy(frame + 20, voice + 14, 13);

	payload[0] = (index == 0) ? MMDVM_DMR_SYNC_AUDIO : index;
	memcpy(payload + 1, frame, MMDVM_DMR_FRAME_LEN);
	if (heard && hotspot_rssi_byte(rssi_dbm, &raw)) {
		payload[len++] = 0;
		payload[len++] = raw;
	}
	mmdvm_send(up_cmd(k, false), payload, len);
}

static bool data_with_crc(uint8_t kind)
{
	return (kind == DMR_BURST_PI_HEADER) || (kind == DMR_BURST_CSBK) || (kind == DMR_BURST_MBC_HEADER) ||
	       (kind == DMR_BURST_DATA_HEADER);
}

void hotspotSlotBurst(hotspotSlot_t *r, int k, const dmr_burst_t *b, uint32_t now_ms)
{
	hotspotUplinkBurst_t u;
	uint8_t fill = 0;

	r->last_ms = now_ms;
	r->heard = true;
	memset(&u, 0, sizeof(u));
	u.ms = now_ms;
	switch (b->kind) {
	case DMR_BURST_VOICE:
		u.kind = HOTSPOT_UPLINK_VOICE;
		u.seq = b->voice_index;
		u.flags = (uint8_t)((b->lc_valid ? HOTSPOT_UPLINK_FLAG_LC_VALID : 0) |
				    (((b->voice_index == 0) || b->crc_ok) ? HOTSPOT_UPLINK_FLAG_SYNC_OK : 0));
		memcpy(u.lc, b->lc, LC_LEN);
		memcpy(u.ambe, b->payload, HOTSPOT_UPLINK_AMBE_LENGTH);
		if (b->lc_valid) {
			uint8_t flco = b->lc[0] & 0x3F;

			if ((flco >= DMR_FLCO_TA_HEADER) && (flco <= DMR_FLCO_GPS)) {
				memcpy(r->alias_lc, b->lc, LC_LEN);
				r->alias_pending = true;
			}
		}
		break;
	case DMR_BURST_VOICE_LC_HEADER:
	case DMR_BURST_TERMINATOR_LC:
		if (!b->crc_ok) {
			return;
		}
		u.kind = (b->kind == DMR_BURST_VOICE_LC_HEADER) ? HOTSPOT_UPLINK_HEADER : HOTSPOT_UPLINK_TERMINATOR;
		u.flags = HOTSPOT_UPLINK_FLAG_LC_VALID;
		memcpy(u.lc, b->lc_valid ? b->lc : b->payload, LC_LEN);
		break;
	default:
		if ((b->kind > DMR_BURST_RATE_1_DATA) || (b->kind == DMR_BURST_IDLE)) {
			return;
		}
		if (data_with_crc(b->kind) && !b->crc_ok) {
			return;
		}
		r->data_ms = now_ms;
		if (hotspotUplinkData(&r->up, b->kind, now_ms)) {
			hotspot_up_data(up_cmd(k, false), b->kind, b->payload, b->len,
					(b->rssi_dbm != 0) ? b->rssi_dbm : 0);
		}
		return;
	}

	uint8_t actions = hotspotUplinkBurst(&r->up, &u, &fill);
	const uint8_t *lc = hotspotUplinkLc(&r->up);

	if (actions & HOTSPOT_UP_LOST) {
		hotspotSlotLost(r, k);
	}
	if (actions & HOTSPOT_UP_HEADER) {
		hotspot_up_lc(up_cmd(k, false), DMR_TYPE_VOICE_LC_HEADER, lc,
			      (b->kind == DMR_BURST_VOICE_LC_HEADER) ? b->rssi_dbm : 0);
		dmr_emb_lc_encode(lc, r->sf_frag);
	}
	if (actions & HOTSPOT_UP_FILL) {
		for (uint8_t i = fill, n = 0; (i != u.seq) && (n < 6); i = (uint8_t)((i + 1) % 6), n++) {
			rx_voice(k, r, HOTSPOT_UPLINK_SILENCE, i, 0, false);
		}
	}
	if (actions & HOTSPOT_UP_VOICE) {
		rx_voice(k, r, u.ambe, u.seq, b->rssi_dbm, true);
	}
	if (actions & HOTSPOT_UP_TERMINATOR) {
		hotspot_up_lc(up_cmd(k, false), DMR_TYPE_TERMINATOR_LC, lc, b->rssi_dbm);
		r->alias_pending = false;
	}
}

void hotspotSlotPoll(hotspotSlot_t *r, int k, uint32_t now_ms)
{
	if (hotspotUplinkLost(&r->up, now_ms)) {
		hotspotSlotLost(r, k);
	}
}
