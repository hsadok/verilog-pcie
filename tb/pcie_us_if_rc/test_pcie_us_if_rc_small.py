#!/usr/bin/env python
"""
Stress test for pcie_us_if_rc (+ pcie_tlp_fifo) with many small completions,
as produced when one device reads small messages from two completers
(peer: 96 B single completions; host: 64 B RCB-split completions).
Random source idle and sink backpressure.  Checks every TLP and the
sop/eop framing of the output stream.
"""

import logging
import os
import random
import sys

import cocotb_test.simulator

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

from cocotbext.axi import AxiStreamBus
from cocotbext.pcie.core.tlp import Tlp, TlpType
from cocotbext.pcie.xilinx.us.interface import RcSource
from cocotbext.pcie.xilinx.us.tlp import Tlp_us

try:
    from pcie_if import PcieIfSink, PcieIfRxBus
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
    try:
        from pcie_if import PcieIfSink, PcieIfRxBus
    finally:
        del sys.path[0]


def rand_pause(rng, p):
    while True:
        yield 1 if rng.random() < p else 0


@cocotb.test()
async def run_small_stress(dut):
    log = logging.getLogger("cocotb.tb")
    seed = int(os.getenv("SEED", "1"))
    n = int(os.getenv("N_TLPS", "3000"))
    rng = random.Random(seed)

    cocotb.start_soon(Clock(dut.clk, 4, units="ns").start())
    source = RcSource(AxiStreamBus.from_prefix(dut, "s_axis_rc"), dut.clk, dut.rst, segments=len(dut.rx_cpl_tlp_valid_reg))
    sink = PcieIfSink(PcieIfRxBus.from_prefix(dut, "rx_cpl_tlp"), dut.clk, dut.rst)
    source.log.setLevel(logging.WARNING)
    sink.log.setLevel(logging.WARNING)

    dut.rst.setimmediatevalue(0)
    await RisingEdge(dut.clk)
    dut.rst.value = 1
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)

    source.set_pause_generator(rand_pause(rng, int(os.getenv("SRC_IDLE", "20")) / 100))
    sink.set_pause_generator(rand_pause(rng, int(os.getenv("SINK_BP", "30")) / 100))

    framing_errors = []

    async def framing_monitor():
        in_frame = False
        while True:
            await RisingEdge(dut.clk)
            if dut.rx_cpl_tlp_valid.value and dut.rx_cpl_tlp_ready.value:
                sop = int(dut.rx_cpl_tlp_sop.value)
                eop = int(dut.rx_cpl_tlp_eop.value)
                if sop and in_frame:
                    framing_errors.append(f"sop inside open TLP at {cocotb.utils.get_sim_time('ns')} ns")
                if not sop and not in_frame:
                    framing_errors.append(f"continuation beat without open TLP at {cocotb.utils.get_sim_time('ns')} ns")
                in_frame = not eop

    cocotb.start_soon(framing_monitor())

    straddle = [0, 0]

    async def straddle_monitor():
        while True:
            await RisingEdge(dut.clk)
            if dut.s_axis_rc_tvalid.value and dut.s_axis_rc_tready.value:
                u = dut.s_axis_rc_tuser.value.integer
                nsop = bin((u >> 64) & 0xf).count("1")
                if nsop >= 2:
                    straddle[0] += 1
                if nsop >= 3:
                    straddle[1] += 1

    cocotb.start_soon(straddle_monitor())

    sent = []
    for k in range(n):
        kind = rng.random()
        if kind < 0.4:
            nbytes = 96  # peer message, one completion
            la = rng.choice([0, 0x20, 0x40, 0x3c, 0x08])
        elif kind < 0.8:
            # host 68 B split at 64 B RCB
            nbytes = rng.choice([4, 8, 32, 36, 60, 64, 68 - 32])
            la = rng.randrange(0, 128, 4)
        else:
            nbytes = rng.randrange(4, 260, 4)
            la = rng.randrange(0, 128, 4)
        data = bytearray(rng.getrandbits(8) for _ in range(nbytes))
        tlp = Tlp()
        tlp.fmt_type = TlpType.CPL_DATA
        tlp.set_data(data)
        tlp.byte_count = nbytes + rng.choice([0, 0, 4, 32])
        tlp.lower_address = la
        tlp.tag = k % 256
        sent.append(tlp)
        await source.send(Tlp_us(tlp).pack_us_rc())

    for k, tlp in enumerate(sent):
        rx = (await sink.recv()).to_tlp()
        if rx != tlp:
            log.error("TLP %d mismatch:\n sent %s\n got  %s", k, tlp, rx)
            assert False, f"TLP {k} mismatch"

    assert sink.empty()
    assert not framing_errors, framing_errors[:5]
    log.info("%d TLPs OK; input beats with >=2 SOPs: %d, >=3 SOPs: %d", n, straddle[0], straddle[1])


tests_dir = os.path.dirname(__file__)
rtl_dir = os.path.abspath(os.path.join(tests_dir, '..', '..', 'rtl'))


def test_pcie_us_if_rc_small(request):
    dut = "pcie_us_if_rc"
    module = os.path.splitext(os.path.basename(__file__))[0]
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
    parameters['TLP_SEG_COUNT'] = 1
    extra_env = {f'PARAM_{k}': str(v) for k, v in parameters.items()}
    extra_env['SEED'] = os.getenv("SEED", "1")
    sim_build = os.path.join(tests_dir, "sim_build", request.node.name.replace('[', '-').replace(']', ''))
    cocotb_test.simulator.run(
        python_search=[tests_dir],
        verilog_sources=verilog_sources,
        toplevel=dut,
        module=module,
        parameters=parameters,
        sim_build=sim_build,
        extra_env=extra_env,
    )
