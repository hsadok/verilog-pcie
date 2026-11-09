#!/usr/bin/env python
"""
pcie_us_if_cq with a 512-bit straddled CQ stream built beat by beat from the
UltraScale+ integrated block guide (PG213 v1.3), independently of the
cocotbext-pcie CqSource model.

PG213, CQ interface, 512-bit, straddle enabled:
  - A TLP starts at byte lane 0 (is_sop_ptr 2'b00) or byte lane 32 (2'b10);
    a TLP starts at byte lane 32 only if the previous TLP ended before byte
    lane 32 in the same beat. At most two TLPs start in a beat.
  - first_be[3:0] / last_be[11:8] belong to the FIRST TLP starting in the
    beat (valid with is_sop[0]); first_be[7:4] / last_be[15:12] to the second
    one (is_sop[1]). The slot follows start order, not position: a TLP that
    starts at byte lane 32 behind a TLP that ended in the lower half is the
    first TLP starting in that beat, so its byte enables are in slot 0.
  - is_eop0_ptr / is_eop1_ptr give the last Dword of the first / second TLP
    ending in the beat; tkeep is all ones and tlast is not used.

Every TLP is checked field by field (including first/last BE) on the
adapter's output. Writes use random unaligned addresses and lengths so the
byte enables vary, and the stream is forced to contain the case above.
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

from cocotbext.pcie.core.tlp import Tlp, TlpType
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


class Pg213CqDriver:
    """Drives s_axis_cq (512-bit, straddle) following PG213."""

    def __init__(self, dut, rng, idle_p, straddle_p):
        self.dut = dut
        self.rng = rng
        self.idle_p = idle_p
        self.straddle_p = straddle_p
        self.stats = {'beats': 0, 'sop_at_8_alone': 0, 'two_sops': 0}
        dut.s_axis_cq_tvalid.setimmediatevalue(0)
        dut.s_axis_cq_tdata.setimmediatevalue(0)
        dut.s_axis_cq_tkeep.setimmediatevalue(0)
        dut.s_axis_cq_tlast.setimmediatevalue(0)
        dut.s_axis_cq_tuser.setimmediatevalue(0)

    def _beats(self, frames, force_straddle=()):
        def new_beat():
            return {'data': [0] * DW_PER_BEAT, 'be': [0] * DW_PER_BEAT,
                    'sops': [], 'eops': [], 'fbe': [], 'lbe': []}

        beat = new_beat()
        pos = 0          # next free Dword in the beat
        last_end = None  # Dword where the previous TLP ended in this beat
        for idx, f in enumerate(frames):
            # Choose where the TLP starts.
            if pos == 0 and not beat['eops'] and not beat['sops']:
                start = 0
            elif (last_end is not None and last_end < 8 and
                  len(beat['sops']) < 2 and
                  (idx in force_straddle or self.rng.random() < self.straddle_p)):
                start = 8
            else:
                yield beat
                beat = new_beat()
                start = 0
            beat['sops'].append(start)
            beat['fbe'].append(f.first_be & 0xf)
            beat['lbe'].append(f.last_be & 0xf)
            pos = start
            last_end = None
            byte_en = list(f.byte_en) + [0] * (len(f.data) - len(f.byte_en))
            for i, dw in enumerate(f.data):
                if pos == DW_PER_BEAT:
                    yield beat
                    beat = new_beat()
                    pos = 0
                beat['data'][pos] = dw
                beat['be'][pos] = byte_en[i] & 0xf
                pos += 1
            beat['eops'].append(pos - 1)
            last_end = pos - 1
        if beat['sops'] or beat['eops']:
            yield beat

    def _encode(self, beat):
        tdata = 0
        for i, dw in enumerate(beat['data']):
            tdata |= (dw & 0xffffffff) << (32 * i)
        tuser = 0
        for k, (fbe, lbe) in enumerate(zip(beat['fbe'], beat['lbe'])):
            tuser |= fbe << (4 * k)          # first_be[7:0]
            tuser |= lbe << (8 + 4 * k)      # last_be[15:8]
        for i, be in enumerate(beat['be']):
            tuser |= be << (16 + 4 * i)      # byte_en[79:16]
        for k, start in enumerate(beat['sops']):
            tuser |= 1 << (80 + k)           # is_sop[1:0]
            tuser |= (start // 4) << (82 + 2 * k)  # is_sop0/1_ptr
        for k, end in enumerate(beat['eops']):
            tuser |= 1 << (86 + k)           # is_eop[1:0]
            tuser |= end << (88 + 4 * k)     # is_eop0/1_ptr
        if len(beat['sops']) == 1 and beat['sops'][0] == 8:
            self.stats['sop_at_8_alone'] += 1
        if len(beat['sops']) == 2:
            self.stats['two_sops'] += 1
        return tdata, tuser

    async def send(self, frames, force_straddle=()):
        dut = self.dut
        for beat in self._beats(frames, force_straddle):
            while self.rng.random() < self.idle_p:
                dut.s_axis_cq_tvalid.value = 0
                await RisingEdge(dut.clk)
            tdata, tuser = self._encode(beat)
            dut.s_axis_cq_tdata.value = tdata
            dut.s_axis_cq_tkeep.value = (1 << DW_PER_BEAT) - 1
            dut.s_axis_cq_tlast.value = 0
            dut.s_axis_cq_tuser.value = tuser
            dut.s_axis_cq_tvalid.value = 1
            # The beat transfers at the first rising edge that sees tready.
            while True:
                await RisingEdge(dut.clk)
                if dut.s_axis_cq_tready.value:
                    break
            self.stats['beats'] += 1
        dut.s_axis_cq_tvalid.value = 0


def rand_pause(rng, p):
    while True:
        yield 1 if rng.random() < p else 0


async def reset(dut):
    cocotb.start_soon(Clock(dut.clk, 4, units="ns").start())
    dut.rst.setimmediatevalue(0)
    await RisingEdge(dut.clk)
    dut.rst.value = 1
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)


def make_write(rng, addr, nbytes, tag):
    tlp = Tlp()
    tlp.fmt_type = TlpType.MEM_WRITE_64
    tlp.set_addr_be_data(addr, bytearray(rng.getrandbits(8) for _ in range(nbytes)))
    tlp.requester_id = PcieId(rng.randrange(256), rng.randrange(32), rng.randrange(8))
    tlp.tag = tag
    return tlp


def make_read(rng, addr, nbytes, tag):
    tlp = Tlp()
    tlp.fmt_type = TlpType.MEM_READ_64
    tlp.set_addr_be(addr, nbytes)
    tlp.requester_id = PcieId(rng.randrange(256), rng.randrange(32), rng.randrange(8))
    tlp.tag = tag
    return tlp


async def check(dut, sink, tlps, log):
    for k, tlp in enumerate(tlps):
        rx = (await sink.recv()).to_tlp()
        if rx != tlp:
            diff = {f: (getattr(tlp, f), getattr(rx, f)) for f in vars(tlp)
                    if getattr(tlp, f) != getattr(rx, f, None)}
            log.error("TLP %d mismatch (sent, got): %s", k, diff)
            assert False, f"TLP {k} mismatch: {sorted(diff)}"
    assert sink.empty()


async def setup(dut, seed, idle_pct, bp_pct, straddle_pct):
    log = logging.getLogger("cocotb.tb")
    rng = random.Random(seed)
    await reset(dut)
    drv = Pg213CqDriver(dut, rng, idle_pct / 100, straddle_pct / 100)
    sink = PcieIfSink(PcieIfRxBus.from_prefix(dut, "rx_req_tlp"), dut.clk, dut.rst)
    sink.log.setLevel(logging.WARNING)
    sink.set_pause_generator(rand_pause(rng, bp_pct / 100))
    await RisingEdge(dut.clk)
    return log, rng, drv, sink


@cocotb.test()
async def run_directed_second_half_byte_enables(dut):
    """The DCP consumer's workaround case: a write starting at byte lane 32
    behind a TLP that ended in the lower half (the only TLP starting in that
    beat), with 16 B and 8 B payloads and non-trivial byte enables."""
    log, rng, drv, sink = await setup(dut, 1, 0, 0, 0)
    tlps = []
    force = set()
    # A 64 B write is a 4 + 16 = 20 Dword frame: it ends at Dword 3 of its
    # second beat, so the next TLP can start at Dword 8 of that beat.
    for nbytes, addr in [(16, 0x1000), (8, 0x2000), (4, 0x3000), (12, 0x4004),
                         (7, 0x5001), (2, 0x6002), (64, 0x7000), (1, 0x8003)]:
        tlps.append(make_write(rng, 0x100, 64, len(tlps)))
        force.add(len(tlps))
        tlps.append(make_write(rng, addr, nbytes, len(tlps)))
    # Reads too: their byte enables are in the header as well.
    for nbytes, addr in [(4, 0x9000), (8, 0x9104), (6, 0x9201)]:
        tlps.append(make_write(rng, 0x100, 64, len(tlps)))
        force.add(len(tlps))
        tlps.append(make_read(rng, addr, nbytes, len(tlps)))
    frames = [Tlp_us(t).pack_us_cq() for t in tlps]
    cocotb.start_soon(drv.send(frames, force_straddle=force))
    await check(dut, sink, tlps, log)
    log.info("stats %s", drv.stats)
    assert drv.stats['sop_at_8_alone'] >= len(force), drv.stats


@cocotb.test()
async def run_random_stream(dut):
    """Random writes and reads (unaligned addresses, 1..512 B), random
    straddling, source idle and sink backpressure."""
    seed = int(os.getenv("SEED", "1"))
    n = int(os.getenv("N_TLPS", "2000"))
    log, rng, drv, sink = await setup(
        dut, seed, int(os.getenv("SRC_IDLE", "20")),
        int(os.getenv("SINK_BP", "30")), int(os.getenv("STRADDLE", "70")))
    tlps = []
    for k in range(n):
        r = rng.random()
        if r < 0.35:
            nbytes = rng.choice([1, 2, 3, 4, 5, 7, 8, 12, 16])
        elif r < 0.8:
            nbytes = rng.randint(1, 128)
        else:
            nbytes = rng.randint(1, 512)
        if rng.random() < 0.7:
            addr = rng.randrange(0, 1 << 20)       # unaligned: first_be varies
        else:
            addr = rng.randrange(0, 1 << 20, 4)
        # Keep within one 4 KiB page (a TLP may not cross it).
        addr = (addr & ~0xfff) | (addr & 0xfff) % (0x1000 - nbytes + 1)
        if rng.random() < 0.75:
            tlps.append(make_write(rng, addr, nbytes, k % 256))
        else:
            tlps.append(make_read(rng, addr, nbytes, k % 256))
    frames = [Tlp_us(t).pack_us_cq() for t in tlps]
    cocotb.start_soon(drv.send(frames))
    await check(dut, sink, tlps, log)
    log.info("%d TLPs OK, stats %s", n, drv.stats)
    assert drv.stats['sop_at_8_alone'] > 0 and drv.stats['two_sops'] > 0, drv.stats


tests_dir = os.path.dirname(__file__)
rtl_dir = os.path.abspath(os.path.join(tests_dir, '..', '..', 'rtl'))


@pytest.mark.parametrize("tlp_seg_count", [1, 2])
def test_pcie_us_if_cq_pg213(request, tlp_seg_count):
    dut = "pcie_us_if_cq"
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
    parameters['AXIS_PCIE_CQ_USER_WIDTH'] = 183
    parameters['CQ_STRADDLE'] = 1
    parameters['TLP_DATA_WIDTH'] = 512
    parameters['TLP_STRB_WIDTH'] = 16
    parameters['TLP_HDR_WIDTH'] = 128
    parameters['TLP_SEG_COUNT'] = tlp_seg_count

    extra_env = {f'PARAM_{k}': str(v) for k, v in parameters.items()}
    for k in ("SEED", "N_TLPS", "SRC_IDLE", "SINK_BP", "STRADDLE"):
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
