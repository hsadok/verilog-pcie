#!/usr/bin/env python
"""

Copyright (c) 2022 Alex Forencich

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

"""

import itertools
import logging
import os
import random
import subprocess
import sys

import cocotb_test.simulator
import pytest

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge
from cocotb.regression import TestFactory

from cocotbext.pcie.core.tlp import Tlp, TlpType


try:
    from pcie_if import PcieIfSource, PcieIfSink, PcieIfBus, PcieIfFrame
except ImportError:
    # attempt import from current directory
    sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
    try:
        from pcie_if import PcieIfSource, PcieIfSink, PcieIfBus, PcieIfFrame
    finally:
        del sys.path[0]


class TB(object):
    def __init__(self, dut):
        self.dut = dut

        ports = len(dut.pcie_tlp_demux_inst.out_tlp_ready)

        self.log = logging.getLogger("cocotb.tb")
        self.log.setLevel(logging.DEBUG)

        cocotb.start_soon(Clock(dut.clk, 4, units="ns").start())

        self.source = PcieIfSource(PcieIfBus.from_prefix(dut, "in_tlp"), dut.clk, dut.rst)
        self.sink = [PcieIfSink(PcieIfBus.from_prefix(dut, f"out{k:02d}_tlp"), dut.clk, dut.rst) for k in range(ports)]

        dut.enable.setimmediatevalue(0)
        dut.drop.setimmediatevalue(0)
        for k in range(ports):
            getattr(dut, f"out{k:02d}_select").setimmediatevalue(0)

    def set_idle_generator(self, generator=None):
        if generator:
            self.source.set_pause_generator(generator())

    def set_backpressure_generator(self, generator=None):
        if generator:
            for sink in self.sink:
                sink.set_pause_generator(generator())

    async def cycle_reset(self):
        self.dut.rst.setimmediatevalue(0)
        await RisingEdge(self.dut.clk)
        await RisingEdge(self.dut.clk)
        self.dut.rst.value = 1
        await RisingEdge(self.dut.clk)
        await RisingEdge(self.dut.clk)
        self.dut.rst.value = 0
        await RisingEdge(self.dut.clk)
        await RisingEdge(self.dut.clk)


async def run_test(dut, payload_lengths=None, payload_data=None, idle_inserter=None, backpressure_inserter=None, port=0):

    tb = TB(dut)

    seg_count = len(tb.source.bus.valid)
    seq_count = 2**(len(tb.source.bus.seq) // seg_count)

    cur_seq = 1

    await tb.cycle_reset()

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    test_tlps = []
    test_frames = []

    dut.enable.setimmediatevalue(1)
    dut.drop.setimmediatevalue(0)
    for k in range(ports):
        getattr(dut, f"out{k:02d}_select").setimmediatevalue(0)
    getattr(dut, f"out{port:02d}_select").setimmediatevalue(2**seg_count-1)

    for test_data in [payload_data(x) for x in payload_lengths()]:
        test_tlp = Tlp()

        if len(test_data):
            test_tlp.fmt_type = TlpType.MEM_WRITE
            test_tlp.set_addr_be_data(cur_seq*4, test_data)
            test_tlp.requester_id = port
        else:
            test_tlp.fmt_type = TlpType.MEM_READ
            test_tlp.set_addr_be(cur_seq*4, 4)
            test_tlp.requester_id = port

        test_frame = PcieIfFrame.from_tlp(test_tlp)
        test_frame.seq = cur_seq
        test_frame.func_num = port

        test_tlps.append(test_tlp)
        test_frames.append(test_frame)
        await tb.source.send(test_frame)

        cur_seq = (cur_seq + 1) % seq_count

    for test_tlp in test_tlps:
        rx_frame = await tb.sink[port].recv()

        rx_tlp = rx_frame.to_tlp()

        assert rx_tlp == test_tlp

    for sink in tb.sink:
        assert sink.empty()

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_stress_test(dut, payload_lengths=None, payload_data=None, idle_inserter=None, backpressure_inserter=None, port=0):

    tb = TB(dut)

    seg_count = len(tb.source.bus.valid)
    seq_count = 2**(len(tb.source.bus.seq) // seg_count)

    cur_seq = 1

    await tb.cycle_reset()

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    test_tlps = []
    test_frames = []

    dut.enable.setimmediatevalue(1)
    dut.drop.setimmediatevalue(0)
    for k in range(ports):
        getattr(dut, f"out{k:02d}_select").setimmediatevalue(0)
    getattr(dut, f"out{port:02d}_select").setimmediatevalue(2**seg_count-1)

    for k in range(128):
        length = random.randint(1, 512)
        test_tlp = Tlp()
        test_tlp.fmt_type = random.choice([TlpType.MEM_WRITE, TlpType.MEM_READ])
        if test_tlp.fmt_type == TlpType.MEM_WRITE:
            test_data = bytearray(itertools.islice(itertools.cycle(range(256)), length))
            test_tlp.set_addr_be_data(cur_seq*4, test_data)
            test_tlp.requester_id = port
        elif test_tlp.fmt_type == TlpType.MEM_READ:
            test_tlp.set_addr_be(cur_seq*4, length)
            test_tlp.tag = cur_seq
            test_tlp.requester_id = port

        test_frame = PcieIfFrame.from_tlp(test_tlp)
        test_frame.seq = cur_seq
        test_frame.func_num = port

        test_tlps.append(test_tlp)
        test_frames.append(test_frame)
        await tb.source.send(test_frame)

        cur_seq = (cur_seq + 1) % seq_count

    for test_tlp in test_tlps:
        rx_frame = await tb.sink[port].recv()

        rx_tlp = rx_frame.to_tlp()

        assert rx_tlp == test_tlp

    for sink in tb.sink:
        assert sink.empty()

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_test_port_switching(dut, idle_inserter=None, backpressure_inserter=None):
    """Test routing TLPs to each port sequentially.

    Sends a batch of TLPs to each port, drains, then moves to the next port.
    Validates that the combinational sel_port logic correctly routes to every port.
    """
    tb = TB(dut)

    seg_count = len(tb.source.bus.valid)
    seq_count = 2**(len(tb.source.bus.seq) // seg_count)

    cur_seq = 1

    await tb.cycle_reset()

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    dut.enable.setimmediatevalue(1)
    dut.drop.setimmediatevalue(0)

    # Test each port with various TLP sizes
    lengths = [4, 64, 128, 256]
    for port in range(ports):
        # Configure select for this port
        for k in range(ports):
            getattr(dut, f"out{k:02d}_select").setimmediatevalue(0)
        getattr(dut, f"out{port:02d}_select").setimmediatevalue(2**seg_count - 1)

        test_tlps = []
        for length in lengths:
            test_data = bytearray(itertools.islice(itertools.cycle(range(256)), length))

            test_tlp = Tlp()
            test_tlp.fmt_type = TlpType.MEM_WRITE
            test_tlp.set_addr_be_data(cur_seq * 4, test_data)
            test_tlp.requester_id = port

            test_frame = PcieIfFrame.from_tlp(test_tlp)
            test_frame.seq = cur_seq
            test_frame.func_num = port

            test_tlps.append(test_tlp)
            await tb.source.send(test_frame)

            cur_seq = (cur_seq + 1) % seq_count

        # Verify all TLPs arrived at the correct port
        for test_tlp in test_tlps:
            rx_frame = await tb.sink[port].recv()
            rx_tlp = rx_frame.to_tlp()
            assert rx_tlp == test_tlp, f"TLP mismatch on port {port}"

        # Verify no other port received anything
        for k in range(ports):
            if k != port:
                assert tb.sink[k].empty(), f"Port {k} should be empty when sending to port {port}"

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_test_drop(dut, idle_inserter=None, backpressure_inserter=None):
    """Test the drop signal: TLPs with drop asserted should be silently discarded.

    Three phases: send TLPs normally, then with drop=1 (should be discarded),
    then normally again. Verifies only non-dropped TLPs arrive.
    """
    tb = TB(dut)

    seg_count = len(tb.source.bus.valid)
    seq_count = 2**(len(tb.source.bus.seq) // seg_count)

    cur_seq = 1

    await tb.cycle_reset()

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    port = 0
    dut.enable.setimmediatevalue(1)
    dut.drop.setimmediatevalue(0)
    for k in range(ports):
        getattr(dut, f"out{k:02d}_select").setimmediatevalue(0)
    getattr(dut, f"out{port:02d}_select").setimmediatevalue(2**seg_count - 1)

    # Phase 1: Send 3 TLPs normally (drop=0)
    phase1_tlps = []
    for i in range(3):
        length = 4 + i * 32
        test_data = bytearray(itertools.islice(itertools.cycle(range(256)), length))

        test_tlp = Tlp()
        test_tlp.fmt_type = TlpType.MEM_WRITE
        test_tlp.set_addr_be_data(cur_seq * 4, test_data)
        test_tlp.requester_id = port

        test_frame = PcieIfFrame.from_tlp(test_tlp)
        test_frame.seq = cur_seq
        test_frame.func_num = port

        phase1_tlps.append(test_tlp)
        await tb.source.send(test_frame)
        cur_seq = (cur_seq + 1) % seq_count

    # Verify phase 1
    for test_tlp in phase1_tlps:
        rx_frame = await tb.sink[port].recv()
        rx_tlp = rx_frame.to_tlp()
        assert rx_tlp == test_tlp, "Phase 1 TLP mismatch"

    # Phase 2: Set drop=1, send 3 TLPs (should be dropped)
    dut.drop.setimmediatevalue(1)
    await RisingEdge(dut.clk)

    for i in range(3):
        length = 4 + i * 32
        test_data = bytearray(itertools.islice(itertools.cycle(range(256)), length))

        test_tlp = Tlp()
        test_tlp.fmt_type = TlpType.MEM_WRITE
        test_tlp.set_addr_be_data(cur_seq * 4, test_data)
        test_tlp.requester_id = port

        test_frame = PcieIfFrame.from_tlp(test_tlp)
        test_frame.seq = cur_seq
        test_frame.func_num = port

        await tb.source.send(test_frame)
        cur_seq = (cur_seq + 1) % seq_count

    # Wait for all dropped frames to drain
    for _ in range(100):
        await RisingEdge(dut.clk)

    # Verify nothing arrived (all dropped)
    assert tb.sink[port].empty(), "Dropped TLPs should not appear on output"

    # Phase 3: Set drop=0, send 3 more TLPs normally
    dut.drop.setimmediatevalue(0)
    await RisingEdge(dut.clk)

    phase3_tlps = []
    for i in range(3):
        length = 8 + i * 64
        test_data = bytearray(itertools.islice(itertools.cycle(range(256)), length))

        test_tlp = Tlp()
        test_tlp.fmt_type = TlpType.MEM_WRITE
        test_tlp.set_addr_be_data(cur_seq * 4, test_data)
        test_tlp.requester_id = port

        test_frame = PcieIfFrame.from_tlp(test_tlp)
        test_frame.seq = cur_seq
        test_frame.func_num = port

        phase3_tlps.append(test_tlp)
        await tb.source.send(test_frame)
        cur_seq = (cur_seq + 1) % seq_count

    # Verify phase 3
    for test_tlp in phase3_tlps:
        rx_frame = await tb.sink[port].recv()
        rx_tlp = rx_frame.to_tlp()
        assert rx_tlp == test_tlp, "Phase 3 TLP mismatch"

    for sink in tb.sink:
        assert sink.empty()

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_test_enable(dut, idle_inserter=None, backpressure_inserter=None):
    """Test that the enable signal gates all output.

    When enable is deasserted, no TLPs should pass through. When re-enabled,
    TLPs should flow normally.
    """
    tb = TB(dut)

    seg_count = len(tb.source.bus.valid)
    seq_count = 2**(len(tb.source.bus.seq) // seg_count)

    cur_seq = 1

    await tb.cycle_reset()

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    port = 0
    dut.drop.setimmediatevalue(0)
    for k in range(ports):
        getattr(dut, f"out{k:02d}_select").setimmediatevalue(0)
    getattr(dut, f"out{port:02d}_select").setimmediatevalue(2**seg_count - 1)

    # Phase 1: Enabled - send and verify a TLP
    dut.enable.setimmediatevalue(1)

    test_data = bytearray([0xAA] * 64)
    test_tlp = Tlp()
    test_tlp.fmt_type = TlpType.MEM_WRITE
    test_tlp.set_addr_be_data(cur_seq * 4, test_data)
    test_tlp.requester_id = port

    test_frame = PcieIfFrame.from_tlp(test_tlp)
    test_frame.seq = cur_seq
    test_frame.func_num = port
    cur_seq = (cur_seq + 1) % seq_count

    await tb.source.send(test_frame)

    rx_frame = await tb.sink[port].recv()
    rx_tlp = rx_frame.to_tlp()
    assert rx_tlp == test_tlp

    # Phase 2: Disabled - send and check nothing arrives (with timeout)
    dut.enable.setimmediatevalue(0)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)

    test_data2 = bytearray([0xBB] * 64)
    test_tlp2 = Tlp()
    test_tlp2.fmt_type = TlpType.MEM_WRITE
    test_tlp2.set_addr_be_data(cur_seq * 4, test_data2)
    test_tlp2.requester_id = port

    test_frame2 = PcieIfFrame.from_tlp(test_tlp2)
    test_frame2.seq = cur_seq
    test_frame2.func_num = port
    cur_seq = (cur_seq + 1) % seq_count

    await tb.source.send(test_frame2)

    # Wait a generous amount of time
    for _ in range(50):
        await RisingEdge(dut.clk)

    # Nothing should have arrived
    assert tb.sink[port].empty(), "TLP arrived despite enable=0"

    # Phase 3: Re-enable - the pending TLP should flow through
    dut.enable.setimmediatevalue(1)

    rx_frame2 = await tb.sink[port].recv()
    rx_tlp2 = rx_frame2.to_tlp()
    assert rx_tlp2 == test_tlp2

    for sink in tb.sink:
        assert sink.empty()

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_test_back_to_back_multiport(dut, idle_inserter=None, backpressure_inserter=None):
    """Stress test: send many TLPs to each port with various sizes and types.

    Tests each port with a burst of mixed read/write TLPs to exercise the
    frame tracking and combinational routing under heavy load.
    """
    tb = TB(dut)

    seg_count = len(tb.source.bus.valid)
    seq_count = 2**(len(tb.source.bus.seq) // seg_count)

    cur_seq = 1

    await tb.cycle_reset()

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    dut.enable.setimmediatevalue(1)
    dut.drop.setimmediatevalue(0)

    random.seed(99)

    for port in range(ports):
        # Configure select for this port
        for p in range(ports):
            getattr(dut, f"out{p:02d}_select").setimmediatevalue(0)
        getattr(dut, f"out{port:02d}_select").setimmediatevalue(2**seg_count - 1)

        test_tlps = []
        for k in range(16):
            length = random.randint(1, 128)
            test_tlp = Tlp()
            test_tlp.fmt_type = random.choice([TlpType.MEM_WRITE, TlpType.MEM_READ])
            if test_tlp.fmt_type == TlpType.MEM_WRITE:
                test_data = bytearray(itertools.islice(itertools.cycle(range(256)), length))
                test_tlp.set_addr_be_data(cur_seq * 4, test_data)
                test_tlp.requester_id = port
            elif test_tlp.fmt_type == TlpType.MEM_READ:
                test_tlp.set_addr_be(cur_seq * 4, length)
                test_tlp.tag = cur_seq
                test_tlp.requester_id = port

            test_frame = PcieIfFrame.from_tlp(test_tlp)
            test_frame.seq = cur_seq
            test_frame.func_num = port

            test_tlps.append(test_tlp)
            await tb.source.send(test_frame)

            cur_seq = (cur_seq + 1) % seq_count

        # Verify all TLPs arrived at the correct port
        for test_tlp in test_tlps:
            rx_frame = await tb.sink[port].recv()
            rx_tlp = rx_frame.to_tlp()
            assert rx_tlp == test_tlp, f"TLP mismatch on port {port}"

        # Verify other ports are empty
        for k in range(ports):
            if k != port:
                assert tb.sink[k].empty(), f"Port {k} should be empty"

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


def cycle_pause():
    return itertools.cycle([1, 1, 1, 0])


def size_list():
    return list(range(0, 512+1, 4))+[4]*64


def incrementing_payload(length):
    return bytearray(itertools.islice(itertools.cycle(range(256)), length))


if cocotb.SIM_NAME:

    ports = len(cocotb.top.pcie_tlp_demux_inst.out_tlp_ready)

    factory = TestFactory(run_test)
    factory.add_option("payload_lengths", [size_list])
    factory.add_option("payload_data", [incrementing_payload])
    factory.add_option("idle_inserter", [None, cycle_pause])
    factory.add_option("backpressure_inserter", [None, cycle_pause])
    factory.add_option("port", list(range(ports)))
    factory.generate_tests()

    factory = TestFactory(run_stress_test)
    factory.add_option("idle_inserter", [None, cycle_pause])
    factory.add_option("backpressure_inserter", [None, cycle_pause])
    factory.add_option("port", list(range(ports)))
    factory.generate_tests()

    for test in [
                run_test_port_switching,
                run_test_drop,
                run_test_enable,
                run_test_back_to_back_multiport,
            ]:
        factory = TestFactory(test)
        factory.add_option(("idle_inserter", "backpressure_inserter"), [(None, None), (cycle_pause, cycle_pause)])
        factory.generate_tests()


# cocotb-test

tests_dir = os.path.dirname(__file__)
rtl_dir = os.path.abspath(os.path.join(tests_dir, '..', '..', 'rtl'))


@pytest.mark.parametrize(("pcie_data_width", "tlp_seg_count"),
    [(64, 1), (128, 1), (256, 1), (256, 2), (512, 1), (512, 2), (512, 4)])
@pytest.mark.parametrize("ports", [1, 4])
def test_pcie_tlp_demux(request, pcie_data_width, tlp_seg_count, ports):
    dut = "pcie_tlp_demux"
    wrapper = f"{dut}_wrap_{ports}"
    module = os.path.splitext(os.path.basename(__file__))[0]
    toplevel = wrapper

    # generate wrapper
    wrapper_file = os.path.join(tests_dir, f"{wrapper}.v")
    if not os.path.exists(wrapper_file):
        subprocess.Popen(
            [os.path.join(rtl_dir, f"{dut}_wrap.py"), "-p", f"{ports}"],
            cwd=tests_dir
        ).wait()

    verilog_sources = [
        wrapper_file,
        os.path.join(rtl_dir, f"{dut}.v"),
        os.path.join(rtl_dir, "pcie_tlp_fifo.v"),
        os.path.join(rtl_dir, "pcie_tlp_fifo_raw.v"),
    ]

    parameters = {}

    parameters['TLP_DATA_WIDTH'] = pcie_data_width
    parameters['TLP_STRB_WIDTH'] = parameters['TLP_DATA_WIDTH'] // 32
    parameters['TLP_HDR_WIDTH'] = 128
    parameters['SEQ_NUM_WIDTH'] = 6
    parameters['IN_TLP_SEG_COUNT'] = tlp_seg_count
    parameters['OUT_TLP_SEG_COUNT'] = parameters['IN_TLP_SEG_COUNT']
    parameters['FIFO_ENABLE'] = 1
    parameters['FIFO_DEPTH'] = 4096
    parameters['FIFO_WATERMARK'] = parameters['FIFO_DEPTH'] // 2

    extra_env = {f'PARAM_{k}': str(v) for k, v in parameters.items()}

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
