#!/usr/bin/env python
"""
pcie_us_if_rc with a 512-bit straddled RC stream built beat by beat from the
UltraScale+ integrated block guide (PG213 v1.3), independently of the
cocotbext-pcie RcSource model.

PG213, RC interface, 512-bit, straddle enabled (up to four TLPs per beat):
  - A Completion starts at Dword 0, 4, 8 or 12 (is_sop0..3_ptr 2'b00..2'b11);
    is_sop[k] / is_sop{k}_ptr describe the k-th Completion starting in the
    beat, is_eop[k] / is_eop{k}_ptr the k-th one ending in it.
  - tkeep is all ones and tlast is not used; only is_sop/is_eop frame TLPs.
  - The guide's own example has a beat that starts with no Completion in
    progress and its first Completion at Dword 8 ("Beat 3 has COMPL 3
    starting at Dword offset 8" after COMPL 2 ended in beat 2): the Dwords
    before the first start of such a beat carry no data. LEAD_GAP (percent)
    makes the driver produce such beats.

Every Completion is checked field by field on the adapter's output, and the
output stream's sop/eop framing is checked beat by beat.
"""

import logging
import os
import random
import sys

import cocotb_test.simulator
import pytest

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

from cocotbext.pcie.core.tlp import Tlp, TlpType, CplStatus
from cocotbext.pcie.core.utils import PcieId
from cocotbext.pcie.xilinx.us.tlp import Tlp_us

try:
    from pcie_if import PcieIfSink, PcieIfRxBus
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
    try:
        from pcie_if import PcieIfSink, PcieIfRxBus
    finally:
        del sys.path[0]


DW_PER_BEAT = 16


class Pg213RcDriver:
    """Drives s_axis_rc (512-bit, 4-TLP straddle) following PG213."""

    def __init__(self, dut, rng, idle_p, straddle_p, lead_gap_p, inter_gap_p):
        self.dut = dut
        self.rng = rng
        self.idle_p = idle_p
        self.straddle_p = straddle_p
        self.lead_gap_p = lead_gap_p
        self.inter_gap_p = inter_gap_p
        self.stats = {'beats': 0, 'lead_gap_beats': 0, 'max_sops': 0,
                      'inter_gaps': 0}
        dut.s_axis_rc_tvalid.setimmediatevalue(0)
        dut.s_axis_rc_tdata.setimmediatevalue(0)
        dut.s_axis_rc_tkeep.setimmediatevalue(0)
        dut.s_axis_rc_tlast.setimmediatevalue(0)
        dut.s_axis_rc_tuser.setimmediatevalue(0)

    @staticmethod
    def _new_beat():
        return {'data': [0] * DW_PER_BEAT, 'be': [0] * DW_PER_BEAT,
                'sops': [], 'eops': [], 'lead_gap': False}

    def _beats(self, frames, force_lead_gap=()):
        rng = self.rng
        beat = self._new_beat()
        pos = 0          # next free Dword in the beat
        in_tlp = False   # a TLP continues into the next Dword position
        fresh = True     # beat holds nothing yet
        for idx, f in enumerate(frames):
            # Where the Completion starts: a 4-Dword boundary at or after pos.
            candidates = [p for p in (0, 4, 8, 12) if p >= pos]
            if fresh:
                # Nothing in progress at the start of this beat.
                if idx in force_lead_gap or rng.random() < self.lead_gap_p:
                    start = rng.choice([4, 8, 12])
                    beat['lead_gap'] = True
                else:
                    start = 0
            elif (candidates and len(beat['sops']) < 4 and
                  rng.random() < self.straddle_p):
                start = candidates[0]
                if len(candidates) > 1 and rng.random() < self.inter_gap_p:
                    start = rng.choice(candidates[1:])
                    self.stats['inter_gaps'] += 1
            else:
                yield beat
                beat = self._new_beat()
                fresh = True
                if idx in force_lead_gap or rng.random() < self.lead_gap_p:
                    start = rng.choice([4, 8, 12])
                    beat['lead_gap'] = True
                else:
                    start = 0
            fresh = False
            beat['sops'].append(start)
            pos = start
            byte_en = list(f.byte_en) + [0] * (len(f.data) - len(f.byte_en))
            for i, dw in enumerate(f.data):
                if pos == DW_PER_BEAT:
                    yield beat
                    beat = self._new_beat()
                    pos = 0
                    fresh = False   # this TLP continues in the new beat
                beat['data'][pos] = dw
                beat['be'][pos] = byte_en[i] & 0xf
                pos += 1
            beat['eops'].append(pos - 1)
            # The next Completion starts at the next 4-Dword boundary or later.
            pos = (pos + 3) // 4 * 4
            if pos >= DW_PER_BEAT:
                yield beat
                beat = self._new_beat()
                pos = 0
                fresh = True
        if beat['sops'] or beat['eops']:
            yield beat

    def _encode(self, beat):
        tdata = 0
        tuser = 0
        for i, dw in enumerate(beat['data']):
            tdata |= (dw & 0xffffffff) << (32 * i)
            tuser |= (beat['be'][i] & 0xf) << (4 * i)   # byte_en[63:0]
        for k, start in enumerate(beat['sops']):
            tuser |= 1 << (64 + k)                      # is_sop[3:0]
            tuser |= (start // 4) << (68 + 2 * k)       # is_sop0..3_ptr
        for k, end in enumerate(beat['eops']):
            tuser |= 1 << (76 + k)                      # is_eop[3:0]
            tuser |= end << (80 + 4 * k)                # is_eop0..3_ptr
        if beat['lead_gap']:
            self.stats['lead_gap_beats'] += 1
        self.stats['max_sops'] = max(self.stats['max_sops'], len(beat['sops']))
        return tdata, tuser

    async def send(self, frames, force_lead_gap=()):
        dut = self.dut
        for beat in self._beats(frames, force_lead_gap):
            while self.rng.random() < self.idle_p:
                dut.s_axis_rc_tvalid.value = 0
                await RisingEdge(dut.clk)
            tdata, tuser = self._encode(beat)
            dut.s_axis_rc_tdata.value = tdata
            dut.s_axis_rc_tkeep.value = (1 << DW_PER_BEAT) - 1
            dut.s_axis_rc_tlast.value = 0
            dut.s_axis_rc_tuser.value = tuser
            dut.s_axis_rc_tvalid.value = 1
            # The beat transfers at the first rising edge that sees tready.
            while True:
                await RisingEdge(dut.clk)
                if dut.s_axis_rc_tready.value:
                    break
            self.stats['beats'] += 1
        dut.s_axis_rc_tvalid.value = 0


def rand_pause(rng, p):
    while True:
        yield 1 if rng.random() < p else 0


def make_cpl(rng, k):
    tlp = Tlp()
    r = rng.random()
    if r < 0.08:
        tlp.fmt_type = TlpType.CPL
        tlp.status = rng.choice([CplStatus.UR, CplStatus.CA])
        tlp.byte_count = rng.randrange(4, 4096, 4)  # 0 encodes 4096
    else:
        tlp.fmt_type = TlpType.CPL_DATA
        if r < 0.3:
            nbytes = 4   # one Dword: a 4-Dword frame, so four fit in a beat
        elif r < 0.45:
            nbytes = rng.choice([4, 8, 12, 16, 20, 32, 36, 48, 60, 64, 96])
        elif r < 0.85:
            nbytes = rng.randrange(4, 260, 4)
        else:
            nbytes = rng.randrange(4, 516, 4)
        tlp.set_data(bytearray(rng.getrandbits(8) for _ in range(nbytes)))
        tlp.byte_count = nbytes + rng.choice([0, 0, 0, 4, 64])
    tlp.lower_address = rng.randrange(0, 128)
    tlp.tag = k % 256
    tlp.requester_id = PcieId(rng.randrange(256), rng.randrange(32), rng.randrange(8))
    tlp.completer_id = PcieId(rng.randrange(256), rng.randrange(32), rng.randrange(8))
    return tlp


async def run(dut, seed, n, idle_pct, bp_pct, straddle_pct, lead_gap_pct,
              inter_gap_pct, directed=False):
    log = logging.getLogger("cocotb.tb")
    rng = random.Random(seed)
    cocotb.start_soon(Clock(dut.clk, 4, units="ns").start())
    dut.rst.setimmediatevalue(0)
    await RisingEdge(dut.clk)
    dut.rst.value = 1
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)

    drv = Pg213RcDriver(dut, rng, idle_pct / 100, straddle_pct / 100,
                        lead_gap_pct / 100, inter_gap_pct / 100)
    sink = PcieIfSink(PcieIfRxBus.from_prefix(dut, "rx_cpl_tlp"), dut.clk, dut.rst)
    sink.log.setLevel(logging.WARNING)
    sink.set_pause_generator(rand_pause(rng, bp_pct / 100))

    framing_errors = []

    async def framing_monitor():
        # One frame state, carried across the output segments in order (a
        # TLP can end in segment 0 and the next one start in segment 1).
        segs = len(dut.rx_cpl_tlp_valid)
        in_frame = False
        while True:
            await RisingEdge(dut.clk)
            if not dut.rx_cpl_tlp_ready.value:
                continue
            valid = int(dut.rx_cpl_tlp_valid.value)
            sop = int(dut.rx_cpl_tlp_sop.value)
            eop = int(dut.rx_cpl_tlp_eop.value)
            for s in range(segs):
                if not (valid >> s) & 1:
                    continue
                if (sop >> s) & 1 and in_frame:
                    framing_errors.append(f"sop inside open TLP, seg {s}")
                if not (sop >> s) & 1 and not in_frame:
                    framing_errors.append(f"data without open TLP, seg {s}")
                in_frame = not (eop >> s) & 1

    cocotb.start_soon(framing_monitor())

    tlps = [make_cpl(rng, k) for k in range(n)]
    frames = [Tlp_us(t).pack_us_rc() for t in tlps]
    force = set(range(0, n, 3)) if directed else ()
    cocotb.start_soon(drv.send(frames, force_lead_gap=force))

    for k, tlp in enumerate(tlps):
        rx = (await sink.recv()).to_tlp()
        if rx != tlp:
            diff = {f: (getattr(tlp, f), getattr(rx, f)) for f in vars(tlp)
                    if getattr(tlp, f) != getattr(rx, f, None)}
            log.error("Completion %d mismatch (sent, got): %s", k,
                      {f: v for f, v in diff.items() if f != 'data'} or 'data')
            assert False, f"Completion {k} mismatch: {sorted(diff)}"
    assert sink.empty()
    for _ in range(8):
        await RisingEdge(dut.clk)
    assert not framing_errors, framing_errors[:5]
    log.info("%d Completions OK, stats %s", n, drv.stats)
    return drv.stats


@cocotb.test()
async def run_directed_lead_gap(dut):
    """Beats that start with no Completion in progress and their first
    Completion at Dword 4, 8 or 12 (PG213's beat 3), forced often."""
    stats = await run(dut, 1, 300, 0, 0, 60, 0, 0, directed=True)
    assert stats['lead_gap_beats'] >= 50, stats


@cocotb.test()
async def run_random_stream(dut):
    """Random Completions with random straddling (up to 4 per beat),
    lead gaps, gaps between Completions, source idle and sink backpressure."""
    stats = await run(
        dut, int(os.getenv("SEED", "1")), int(os.getenv("N_TLPS", "3000")),
        int(os.getenv("SRC_IDLE", "20")), int(os.getenv("SINK_BP", "30")),
        int(os.getenv("STRADDLE", "80")), int(os.getenv("LEAD_GAP", "15")),
        int(os.getenv("INTER_GAP", "15")))
    if int(os.getenv("LEAD_GAP", "15")):
        assert stats['lead_gap_beats'] > 0, stats
    assert stats['max_sops'] == 4, stats


@cocotb.test()
async def run_random_stream_no_lead_gap(dut):
    """The same without lead gaps (the stream shape the old adapter handled)."""
    stats = await run(dut, int(os.getenv("SEED", "1")) + 1000, 2000, 20, 30,
                      80, 0, 15)
    assert stats['lead_gap_beats'] == 0 and stats['max_sops'] == 4, stats


tests_dir = os.path.dirname(__file__)
rtl_dir = os.path.abspath(os.path.join(tests_dir, '..', '..', 'rtl'))


@pytest.mark.parametrize("tlp_seg_count", [1, 2])
def test_pcie_us_if_rc_pg213(request, tlp_seg_count):
    dut = "pcie_us_if_rc"
    module = os.path.splitext(os.path.basename(__file__))[0]
    toplevel = dut

    verilog_sources = [
        os.path.join(rtl_dir, f"{dut}.v"),
        os.path.join(rtl_dir, "pcie_tlp_fifo.v"),
        os.path.join(rtl_dir, "pcie_tlp_fifo_raw.v"),
    ]

    parameters = {}
    parameters['AXIS_PCIE_DATA_WIDTH'] = 512
    parameters['AXIS_PCIE_KEEP_WIDTH'] = 16
    parameters['AXIS_PCIE_RC_USER_WIDTH'] = 161
    parameters['RC_STRADDLE'] = 1
    parameters['TLP_DATA_WIDTH'] = 512
    parameters['TLP_STRB_WIDTH'] = 16
    parameters['TLP_HDR_WIDTH'] = 128
    parameters['TLP_SEG_COUNT'] = tlp_seg_count

    extra_env = {f'PARAM_{k}': str(v) for k, v in parameters.items()}
    for k in ("SEED", "N_TLPS", "SRC_IDLE", "SINK_BP", "STRADDLE", "LEAD_GAP",
              "INTER_GAP"):
        if k in os.environ:
            extra_env[k] = os.environ[k]

    sim_build = os.path.join(tests_dir, "sim_build",
        request.node.name.replace('[', '-').replace(']', ''))

    cocotb_test.simulator.run(
        python_search=[tests_dir],
        verilog_sources=verilog_sources,
        toplevel=toplevel,
        module=module,
        parameters=parameters,
        sim_build=sim_build,
        extra_env=extra_env,
    )
