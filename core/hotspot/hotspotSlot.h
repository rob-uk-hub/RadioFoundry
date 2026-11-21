// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
/*
 * One received timeslot as the host wants it (MMDVM DMR_DATA1/2, DMR_LOST1/2): the call tracking and the uplink
 * framing shared by the split duplex RX half and hotspot MS mode. No RTOS, no board globals.
 */

#ifndef RFOUNDRY_HOTSPOTSLOT_H
#define RFOUNDRY_HOTSPOTSLOT_H

#include <stdbool.h>
#include <stdint.h>
#include "hotspot/hotspotUplink.h"
#include "hal_dmr_modem.h"

typedef struct {
	hotspotUplink_t up;
	uint8_t sf_frag[4][4];		// the embedded LC fragments of B-E for the superframe going up
	uint8_t alias_lc[9];		// a talker alias / GPS embedded LC heard: sent in the next superframe
	bool alias_pending;
	uint32_t last_ms;		// last burst heard on this slot
	uint32_t data_ms;		// last data burst heard
	bool heard;
} hotspotSlot_t;

void hotspotSlotReset(hotspotSlot_t *r);
void hotspotSlotLost(hotspotSlot_t *r, int k);		// k: 0 TS1, 1 TS2
// One burst heard on slot k: every supported DMR burst goes up, with the host-expected header / silence fill and
// DMR_LOST from the call tracking
void hotspotSlotBurst(hotspotSlot_t *r, int k, const dmr_burst_t *b, uint32_t now_ms);
void hotspotSlotPoll(hotspotSlot_t *r, int k, uint32_t now_ms);	// the DMR_LOST timeout
static inline bool hotspotSlotHeard(const hotspotSlot_t *r, uint32_t now_ms)
{
	return r->heard && ((uint32_t)(now_ms - r->last_ms) < HOTSPOT_UPLINK_LOST_MS);
}

#endif
