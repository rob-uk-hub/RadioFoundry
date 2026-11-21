// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
/*
 * Hotspot MS mode: an MS on a repeater, both timeslots (MMDVM_CONFIG_MS). Every DMR burst heard on either slot goes
 * up as DMR_DATA1/2, gaps as DMR_LOST1/2; host frames go out on their slot, in order, through the modem's MS
 * outbound path where it has one, otherwise one key-up per burst, or per data list. No host frame is keyed until a
 * BS-synced burst has been heard recently; without one, BS_Dwn_Act wakes the repeater first. Nothing here calls the
 * RTOS or the board.
 */

#ifndef RFOUNDRY_HOTSPOTMS_H
#define RFOUNDRY_HOTSPOTMS_H

#include <stdbool.h>
#include <stdint.h>
#include "hal_dmr_modem.h"

#define HOTSPOT_MS_LC_BYTES	9

// One host frame waiting for its slot, in the pool hotspot.c lends the MS outbound path (hotspot_ms_storage). The
// 8-bit seq orders the two slots' queues; the pool is 64 deep, far below the 128 where the wrap would matter.
typedef struct {
	dmr_tx_burst_t b;
	uint8_t lc[HOTSPOT_MS_LC_BYTES];
	uint8_t seq;
	bool has_lc;
} hotspotMsItem_t;

// After hotspot_init() and mmdvm_init(); storage is hotspot_ms_storage()
void hotspot_ms_init(hotspotMsItem_t *storage);
// The host queue buffer the standard hotspot data path and hotspot MS mode share (one runs at a time)
hotspotMsItem_t *hotspot_ms_storage(void);

#endif
