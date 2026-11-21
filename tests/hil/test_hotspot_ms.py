# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The RadioFoundry Authors
"""Hotspot MS mode (SET_CONFIG flag 0x40, docs/spec/mmdvm-protocol.md 3.6.1): the modem accepts the mode and reports
space for both slots, not the simplex constant 0 for slot 1. No RF is keyed."""
import struct

import pytest

from rfoundry_hil import skip
from hil_config import rig

GET_VERSION, GET_STATUS, SET_CONFIG, SET_FREQ = 0x00, 0x01, 0x02, 0x04


def config(flags=0x40, cc=1):
    return bytes([flags, 0x02, 10, 2, 128, 128, cc, 0, 128, 0, 128, 0, 0, 128, 128] + [0] * 8)


def freq(rx, tx, rf_level=0):
    return bytes([0]) + struct.pack("<II", rx, tx) + bytes([rf_level]) + struct.pack("<I", tx)


@pytest.mark.feature("hotspot_ms")
def test_hotspot_ms_status(radio, pytestconfig):
    if "HOTSPOT_MS" not in radio.features():
        skip("not_built", "built without hotspot MS mode (make CONFIG_HOTSPOT_MS=y)")
    import mmdvm_probe
    hz = round(rig(pytestconfig, "dmr-freq") * 1e6)
    radio.go_home()
    radio.find_channel(hz, "dmr")
    radio.lowest_power()
    with radio.released() as port:
        link = mmdvm_probe.Link(port, dtr=False)
        try:
            assert link.request(GET_VERSION, timeout=2.0)[0] == "reply"
            assert link.request(SET_FREQ, freq(hz, hz, radio.hotspot_rf_level()))[0] == "ack"
            assert link.request(SET_CONFIG, config())[0] == "ack", "hotspot MS mode refused"
            status = None
            for _ in range(6):
                kind, f = link.request(GET_STATUS, timeout=1.0)
                if kind == "reply" and len(f) >= 13:
                    status = f
                    if f[7] and f[8]:
                        break
            assert status is not None, "GET_STATUS did not answer"
            assert status[7] > 0 and status[8] > 0, f"both slot spaces: {status[7]} {status[8]}"
            assert link.request(SET_CONFIG, config(0x80))[0] == "ack"
        finally:
            link.s.close()
    radio.wait_for(lambda x: x.get("hostlink") == "hotspot" and x.get("hs.running") == "1", 3,
                   sections=("hotspot",), what="hotspot MS mode running")
    radio.link.hotspot_exit()
    radio.wait_for(lambda x: x.get("hostlink") == "idle", 5, sections=("hotspot",), what="hotspot left")
