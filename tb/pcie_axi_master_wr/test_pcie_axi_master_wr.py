#!/usr/bin/env python
"""

Copyright (c) 2021 Alex Forencich

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
import re
import sys
from contextlib import contextmanager

import cocotb_test.simulator
import pytest

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer
from cocotb.regression import TestFactory

from cocotbext.pcie.core import RootComplex
from cocotbext.axi import AxiWriteBus, AxiRamWrite


try:
    from pcie_if import PcieIfDevice, PcieIfRxBus
except ImportError:
    # attempt import from current directory
    sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
    try:
        from pcie_if import PcieIfDevice, PcieIfRxBus
    finally:
        del sys.path[0]


@contextmanager
def assert_raises(exc_type, pattern=None):
    try:
        yield
    except exc_type as e:
        if pattern:
            assert re.match(pattern, str(e)), \
                "Correct exception type caught, but message did not match pattern"
        pass
    else:
        raise AssertionError("{} was not raised".format(exc_type.__name__))


class TB(object):
    def __init__(self, dut):
        self.dut = dut

        self.log = logging.getLogger("cocotb.tb")
        self.log.setLevel(logging.DEBUG)

        cocotb.start_soon(Clock(dut.clk, 4, units="ns").start())

        # PCIe
        self.rc = RootComplex()

        self.dev = PcieIfDevice(
            clk=dut.clk,
            rst=dut.rst,

            rx_req_tlp_bus=PcieIfRxBus.from_prefix(dut, "rx_req_tlp")
        )

        self.dev.log.setLevel(logging.DEBUG)

        self.dev.functions[0].configure_bar(0, 16*1024*1024)
        self.dev.functions[0].configure_bar(1, 16*1024, io=True)

        self.rc.make_port().connect(self.dev)

        # AXI
        self.axi_ram = AxiRamWrite(AxiWriteBus.from_prefix(dut, "m_axi"), dut.clk, dut.rst, size=2**16)

        # monitor error outputs
        self.status_error_uncor_asserted = False
        cocotb.start_soon(self._run_monitor_status_error_uncor())

    def set_idle_generator(self, generator=None):
        if generator:
            self.dev.rx_req_tlp_source.set_pause_generator(generator())
            self.axi_ram.b_channel.set_pause_generator(generator())

    def set_backpressure_generator(self, generator=None):
        if generator:
            self.axi_ram.aw_channel.set_pause_generator(generator())
            self.axi_ram.w_channel.set_pause_generator(generator())

    async def _run_monitor_status_error_uncor(self):
        while True:
            await RisingEdge(self.dut.status_error_uncor)
            self.log.info("status_error_uncor (uncorrectable error) was asserted")
            self.status_error_uncor_asserted = True

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


async def run_test_write(dut, idle_inserter=None, backpressure_inserter=None):

    tb = TB(dut)

    byte_lanes = tb.axi_ram.byte_lanes

    pcie_offsets = list(range(byte_lanes))+list(range(4096-byte_lanes, 4096))
    if os.getenv("OFFSET_GROUP") is not None:
        group = int(os.getenv("OFFSET_GROUP"))
        pcie_offsets = pcie_offsets[group::8]

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    await tb.cycle_reset()

    await tb.rc.enumerate()

    dev = tb.rc.find_device(tb.dev.functions[0].pcie_id)
    await dev.enable_device()

    dev_bar0 = dev.bar_window[0]

    for length in list(range(0, byte_lanes*2))+[1024]:
        for pcie_offset in pcie_offsets:
            tb.log.info("length %d, pcie_offset %d", length, pcie_offset)
            pcie_addr = pcie_offset+0x1000
            test_data = bytearray([x % 256 for x in range(length)])

            tb.axi_ram.write(pcie_addr-128, b'\x55'*(len(test_data)+256))

            await dev_bar0.write(pcie_addr, test_data)

            await Timer(length*4+150, 'ns')

            tb.log.debug("%s", tb.axi_ram.hexdump_str((pcie_addr & ~0xf)-16, (((pcie_addr & 0xf)+length-1) & ~0xf)+48, prefix="AXI "))

            assert tb.axi_ram.read(pcie_addr-1, len(test_data)+2) == b'\x55'+test_data+b'\x55'

            assert not tb.status_error_uncor_asserted

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_test_bad_ops(dut, idle_inserter=None, backpressure_inserter=None):

    tb = TB(dut)

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    await tb.cycle_reset()

    await tb.rc.enumerate()

    dev = tb.rc.find_device(tb.dev.functions[0].pcie_id)
    await dev.enable_device()

    dev_bar0 = dev.bar_window[0]
    dev_bar1 = dev.bar_window[1]

    tb.log.info("Test read")

    length = 4
    pcie_addr = 0x1000
    test_data = bytearray([x % 256 for x in range(length)])

    tb.axi_ram.write(pcie_addr-128, b'\x55'*(len(test_data)+256))
    tb.axi_ram.write(pcie_addr, test_data)

    tb.log.debug("%s", tb.axi_ram.hexdump_str((pcie_addr & ~0xf)-16, (((pcie_addr & 0xf)+length-1) & ~0xf)+48, prefix="AXI "))

    with assert_raises(Exception, "Timeout"):
        val = await dev_bar0.read(pcie_addr, len(test_data), timeout=1000, timeout_unit='ns')

    assert tb.status_error_uncor_asserted

    tb.status_error_uncor_asserted = False

    tb.log.info("Test IO write")

    length = 4
    pcie_addr = 0x1000
    test_data = bytearray([x % 256 for x in range(length)])

    tb.axi_ram.write(pcie_addr-128, b'\x55'*(len(test_data)+256))

    with assert_raises(Exception, "Timeout"):
        await dev_bar1.write(pcie_addr, test_data, timeout=1000, timeout_unit='ns')

    await Timer(100, 'ns')

    tb.log.debug("%s", tb.axi_ram.hexdump_str((pcie_addr & ~0xf)-16, (((pcie_addr & 0xf)+length-1) & ~0xf)+48, prefix="AXI "))

    assert tb.axi_ram.read(pcie_addr-1, len(test_data)+2) == b'\x55'*(len(test_data)+2)

    assert tb.status_error_uncor_asserted

    tb.status_error_uncor_asserted = False

    tb.log.info("Test IO read")

    length = 4
    pcie_addr = 0x1000
    test_data = bytearray([x % 256 for x in range(length)])

    tb.axi_ram.write(pcie_addr-128, b'\x55'*(len(test_data)+256))
    tb.axi_ram.write(pcie_addr, test_data)

    tb.log.debug("%s", tb.axi_ram.hexdump_str((pcie_addr & ~0xf)-16, (((pcie_addr & 0xf)+length-1) & ~0xf)+48, prefix="AXI "))

    with assert_raises(Exception, "Timeout"):
        val = await dev_bar1.read(pcie_addr, len(test_data), timeout=1000, timeout_unit='ns')

    assert tb.status_error_uncor_asserted

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_test_write_aligned_no_extra_cycle(dut, idle_inserter=None, backpressure_inserter=None):
    """Test that aligned writes don't waste extra cycles.

    For an aligned write TLP, after the SOP beat is accepted, the module should
    keep rx_req_tlp_ready HIGH for all subsequent data beats. If it drops ready
    for even one cycle while the source has valid data, that is a wasted cycle.

    Only run with no idle/backpressure (the _001 variant) to measure raw performance.
    """
    if idle_inserter is not None or backpressure_inserter is not None:
        # This test only makes sense without idle/backpressure
        return

    tb = TB(dut)

    byte_lanes = tb.axi_ram.byte_lanes

    # No idle insertion or backpressure - we want to measure raw throughput
    await tb.cycle_reset()

    await tb.rc.enumerate()

    dev = tb.rc.find_device(tb.dev.functions[0].pcie_id)
    await dev.enable_device()

    dev_bar0 = dev.bar_window[0]

    for length in [byte_lanes * 2, byte_lanes * 4, 256]:
        if length < byte_lanes * 2:
            continue

        pcie_addr = 0x1000  # aligned to bus width
        test_data = bytearray([x % 256 for x in range(length)])

        tb.axi_ram.write(pcie_addr - 128, b'\x55' * (len(test_data) + 256))

        # Track stall cycles across ALL TLPs for this write operation
        stall_info = {
            'in_transfer': False,
            'stall_cycles': 0,
            'total_input_beats': 0,
            'tlp_count': 0,
            'done': False,
        }

        async def monitor_stalls():
            while not stall_info['done']:
                await RisingEdge(dut.clk)

                tlp_valid = int(dut.rx_req_tlp_valid.value)
                tlp_ready = int(dut.rx_req_tlp_ready.value)
                tlp_sop = int(dut.rx_req_tlp_sop.value)
                tlp_eop = int(dut.rx_req_tlp_eop.value)

                if tlp_valid and tlp_ready and tlp_sop:
                    stall_info['in_transfer'] = True
                    stall_info['total_input_beats'] += 1
                    if tlp_eop:
                        stall_info['in_transfer'] = False
                        stall_info['tlp_count'] += 1
                elif stall_info['in_transfer']:
                    if tlp_valid and tlp_ready:
                        stall_info['total_input_beats'] += 1
                        if tlp_eop:
                            stall_info['in_transfer'] = False
                            stall_info['tlp_count'] += 1
                    elif tlp_valid and not tlp_ready:
                        # Source has data but module isn't accepting it - stall!
                        stall_info['stall_cycles'] += 1

        monitor_task = cocotb.start_soon(monitor_stalls())

        await dev_bar0.write(pcie_addr, test_data)
        await Timer(length * 4 + 500, 'ns')

        stall_info['done'] = True
        await RisingEdge(dut.clk)

        monitor_task.kill()

        # Verify data correctness
        assert tb.axi_ram.read(pcie_addr - 1, len(test_data) + 2) == b'\x55' + test_data + b'\x55'
        assert not tb.status_error_uncor_asserted

        tb.log.info("Aligned write %dB: %d TLPs, %d input beats, %d stall cycles",
                     length, stall_info['tlp_count'], stall_info['total_input_beats'],
                     stall_info['stall_cycles'])

        # For aligned writes with no backpressure, there should be zero stall cycles
        assert stall_info['stall_cycles'] == 0, \
            f"Aligned write of {length}B had {stall_info['stall_cycles']} stall cycle(s) - module wastes cycles!"

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_test_back_to_back_writes(dut, idle_inserter=None, backpressure_inserter=None):
    """Test back-to-back write TLPs of various sizes.

    Verifies that the write module handles rapid consecutive writes correctly,
    ensuring the state machine correctly transitions from one TLP to the next.
    """
    tb = TB(dut)

    byte_lanes = tb.axi_ram.byte_lanes

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    await tb.cycle_reset()

    await tb.rc.enumerate()

    dev = tb.rc.find_device(tb.dev.functions[0].pcie_id)
    await dev.enable_device()

    dev_bar0 = dev.bar_window[0]

    # Issue multiple writes without waiting for completion
    write_configs = [
        (0x1000, 4),               # single DW (awlen==0 path)
        (0x2000, byte_lanes),      # exactly one beat
        (0x3000, byte_lanes * 2),  # two beats (multi-beat optimization)
        (0x4000, byte_lanes * 4),  # four beats
        (0x5000, 1),               # single byte
        (0x6000, byte_lanes + 1),  # one beat + 1 byte overflow
    ]

    expected = {}
    for pcie_addr, length in write_configs:
        test_data = bytearray([(x + pcie_addr) % 256 for x in range(length)])
        tb.axi_ram.write(pcie_addr - 128, b'\x55' * (length + 256))
        expected[pcie_addr] = test_data
        await dev_bar0.write(pcie_addr, test_data)

    await Timer(3000, 'ns')

    for pcie_addr, length in write_configs:
        test_data = expected[pcie_addr]
        tb.log.info("Verifying back-to-back write: addr=0x%04x, len=%d", pcie_addr, length)
        assert tb.axi_ram.read(pcie_addr - 1, len(test_data) + 2) == b'\x55' + test_data + b'\x55', \
            f"Data mismatch at addr=0x{pcie_addr:04x}, len={length}"

    assert not tb.status_error_uncor_asserted

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_test_write_single_dword(dut, idle_inserter=None, backpressure_inserter=None):
    """Test single-DWORD writes (awlen==0 fallback path).

    These exercise the fallback path in STATE_IDLE where first_cycle is used
    instead of the multi-beat optimization (awlen > 0 branch).
    """
    tb = TB(dut)

    byte_lanes = tb.axi_ram.byte_lanes

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    await tb.cycle_reset()

    await tb.rc.enumerate()

    dev = tb.rc.find_device(tb.dev.functions[0].pcie_id)
    await dev.enable_device()

    dev_bar0 = dev.bar_window[0]

    # Single-DWORD writes at various sub-DWORD lengths
    for length in [1, 2, 3, 4]:
        for offset in [0, 1, 2, 3]:
            if offset + length > 4:
                continue
            pcie_addr = 0x1000 + offset
            test_data = bytearray([(x + offset + length) % 256 for x in range(length)])

            tb.axi_ram.write(pcie_addr - 128, b'\x55' * (length + 256))

            tb.log.info("Single-DW write: len=%d, offset=%d", length, offset)
            await dev_bar0.write(pcie_addr, test_data)
            await Timer(200, 'ns')

            assert tb.axi_ram.read(pcie_addr - 1, length + 2) == b'\x55' + test_data + b'\x55', \
                f"Single-DW write mismatch: len={length}, offset={offset}"

    assert not tb.status_error_uncor_asserted

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_test_large_write(dut, idle_inserter=None, backpressure_inserter=None):
    """Test large writes that span multiple TLPs.

    Writes larger than the max payload size get split into multiple TLPs.
    This verifies the write module correctly handles multiple consecutive TLPs
    forming a single large transfer, testing the state machine cycling through
    STATE_IDLE -> STATE_TRANSFER -> STATE_IDLE -> STATE_TRANSFER etc.
    """
    tb = TB(dut)

    byte_lanes = tb.axi_ram.byte_lanes

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    await tb.cycle_reset()

    await tb.rc.enumerate()

    dev = tb.rc.find_device(tb.dev.functions[0].pcie_id)
    await dev.enable_device()

    dev_bar0 = dev.bar_window[0]

    for length in [128, 256, 512, 1024]:
        pcie_addr = 0x1000
        test_data = bytearray([x % 256 for x in range(length)])

        tb.axi_ram.write(pcie_addr - 128, b'\x55' * (length + 256))

        tb.log.info("Large write: len=%d", length)
        await dev_bar0.write(pcie_addr, test_data)
        await Timer(length * 4 + 500, 'ns')

        assert tb.axi_ram.read(pcie_addr - 1, length + 2) == b'\x55' + test_data + b'\x55', \
            f"Large write data mismatch at len={length}"

    assert not tb.status_error_uncor_asserted

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_test_unaligned_multibeat(dut, idle_inserter=None, backpressure_inserter=None):
    """Test multi-beat writes at various unaligned offsets.

    The first-beat optimization in STATE_IDLE computes strobe masks based on
    the address offset. This test exercises that logic with offsets that cause
    the first and last beats to be partial, ensuring strobe computation is correct.
    """
    tb = TB(dut)

    byte_lanes = tb.axi_ram.byte_lanes

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    await tb.cycle_reset()

    await tb.rc.enumerate()

    dev = tb.rc.find_device(tb.dev.functions[0].pcie_id)
    await dev.enable_device()

    dev_bar0 = dev.bar_window[0]

    # Test multi-beat writes at each sub-bus-width offset
    for offset in range(1, byte_lanes):
        for num_beats in [2, 3, 4]:
            length = byte_lanes * num_beats - offset
            if length < byte_lanes + 1:
                continue  # must be multi-beat
            pcie_addr = 0x1000 + offset
            test_data = bytearray([(x + offset + num_beats) % 256 for x in range(length)])

            tb.axi_ram.write(pcie_addr - 128, b'\x55' * (length + 256))

            tb.log.info("Unaligned multi-beat: offset=%d, beats=%d, len=%d", offset, num_beats, length)
            await dev_bar0.write(pcie_addr, test_data)
            await Timer(length * 4 + 300, 'ns')

            assert tb.axi_ram.read(pcie_addr - 1, length + 2) == b'\x55' + test_data + b'\x55', \
                f"Unaligned multi-beat mismatch: offset={offset}, beats={num_beats}, len={length}"

    assert not tb.status_error_uncor_asserted

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_test_stress(dut, idle_inserter=None, backpressure_inserter=None):
    """Stress test with random write sizes and addresses.

    Sends many writes with random lengths and offsets to stress the state machine
    transitions, strobe logic, and burst handling under varied conditions.
    """
    tb = TB(dut)

    byte_lanes = tb.axi_ram.byte_lanes

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    await tb.cycle_reset()

    await tb.rc.enumerate()

    dev = tb.rc.find_device(tb.dev.functions[0].pcie_id)
    await dev.enable_device()

    dev_bar0 = dev.bar_window[0]

    random.seed(42)

    write_ops = []
    for i in range(30):
        length = random.randint(1, 128)
        offset = random.randint(0, byte_lanes - 1)
        pcie_addr = 0x200 + i * 0x400 + offset  # spread out to avoid overlaps
        test_data = bytearray([random.randint(0, 255) for _ in range(length)])

        tb.axi_ram.write(pcie_addr - 128, b'\x55' * (length + 256))
        write_ops.append((pcie_addr, test_data))

        await dev_bar0.write(pcie_addr, test_data)

    await Timer(10000, 'ns')

    for pcie_addr, test_data in write_ops:
        length = len(test_data)
        assert tb.axi_ram.read(pcie_addr - 1, length + 2) == b'\x55' + test_data + b'\x55', \
            f"Stress write mismatch at addr=0x{pcie_addr:04x}, len={length}"

    assert not tb.status_error_uncor_asserted

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_test_aw_backpressure_no_bubble(dut, idle_inserter=None, backpressure_inserter=None):
    """Test that AW channel backpressure doesn't create TLP input bubbles.

    Applies backpressure ONLY on the AW channel (not the W channel) and
    verifies that the write data path has zero stall cycles. This validates
    that the AW output FIFO properly decouples AW channel readiness from
    TLP acceptance.

    Only run without idle/backpressure (the _001 variant) to isolate AW effects.
    """
    if idle_inserter is not None or backpressure_inserter is not None:
        return

    tb = TB(dut)

    byte_lanes = tb.axi_ram.byte_lanes

    await tb.cycle_reset()

    await tb.rc.enumerate()

    dev = tb.rc.find_device(tb.dev.functions[0].pcie_id)
    await dev.enable_device()

    dev_bar0 = dev.bar_window[0]

    # Apply AW-only backpressure: accept every other cycle
    tb.axi_ram.aw_channel.set_pause_generator(itertools.cycle([1, 1, 0, 0]))

    for length in [byte_lanes * 2, byte_lanes * 4, 256]:
        pcie_addr = 0x1000
        test_data = bytearray([x % 256 for x in range(length)])

        tb.axi_ram.write(pcie_addr - 128, b'\x55' * (len(test_data) + 256))

        stall_info = {
            'in_transfer': False,
            'stall_cycles': 0,
            'total_input_beats': 0,
            'tlp_count': 0,
            'done': False,
        }

        async def monitor_stalls():
            while not stall_info['done']:
                await RisingEdge(dut.clk)

                tlp_valid = int(dut.rx_req_tlp_valid.value)
                tlp_ready = int(dut.rx_req_tlp_ready.value)
                tlp_sop = int(dut.rx_req_tlp_sop.value)
                tlp_eop = int(dut.rx_req_tlp_eop.value)

                if tlp_valid and tlp_ready and tlp_sop:
                    stall_info['in_transfer'] = True
                    stall_info['total_input_beats'] += 1
                    if tlp_eop:
                        stall_info['in_transfer'] = False
                        stall_info['tlp_count'] += 1
                elif stall_info['in_transfer']:
                    if tlp_valid and tlp_ready:
                        stall_info['total_input_beats'] += 1
                        if tlp_eop:
                            stall_info['in_transfer'] = False
                            stall_info['tlp_count'] += 1
                    elif tlp_valid and not tlp_ready:
                        stall_info['stall_cycles'] += 1

        monitor_task = cocotb.start_soon(monitor_stalls())

        await dev_bar0.write(pcie_addr, test_data)
        await Timer(length * 4 + 500, 'ns')

        stall_info['done'] = True
        await RisingEdge(dut.clk)

        monitor_task.kill()

        assert tb.axi_ram.read(pcie_addr - 1, len(test_data) + 2) == b'\x55' + test_data + b'\x55'
        assert not tb.status_error_uncor_asserted

        tb.log.info("AW backpressure write %dB: %d TLPs, %d input beats, %d stall cycles",
                     length, stall_info['tlp_count'], stall_info['total_input_beats'],
                     stall_info['stall_cycles'])

        assert stall_info['stall_cycles'] == 0, \
            f"AW backpressure write of {length}B had {stall_info['stall_cycles']} stall cycle(s) " \
            f"- AW FIFO should decouple AW channel from TLP acceptance!"

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


def cycle_pause():
    return itertools.cycle([1, 1, 1, 0])


if cocotb.SIM_NAME:

    for test in [
                run_test_write,
                run_test_bad_ops,
                run_test_write_aligned_no_extra_cycle,
                run_test_back_to_back_writes,
                run_test_write_single_dword,
                run_test_large_write,
                run_test_unaligned_multibeat,
                run_test_stress,
                run_test_aw_backpressure_no_bubble,
            ]:

        factory = TestFactory(test)
        factory.add_option(("idle_inserter", "backpressure_inserter"), [(None, None), (cycle_pause, cycle_pause)])
        factory.generate_tests()


# cocotb-test

tests_dir = os.path.dirname(__file__)
rtl_dir = os.path.abspath(os.path.join(tests_dir, '..', '..', 'rtl'))


@pytest.mark.parametrize("offset_group", list(range(8)))
@pytest.mark.parametrize("pcie_data_width", [64, 128])
def test_pcie_axi_master_wr(request, pcie_data_width, offset_group):
    dut = "pcie_axi_master_wr"
    module = os.path.splitext(os.path.basename(__file__))[0]
    toplevel = dut

    verilog_sources = [
        os.path.join(rtl_dir, f"{dut}.v"),
    ]

    parameters = {}

    parameters['TLP_DATA_WIDTH'] = pcie_data_width
    parameters['TLP_HDR_WIDTH'] = 128
    parameters['TLP_SEG_COUNT'] = 1
    parameters['AXI_DATA_WIDTH'] = parameters['TLP_DATA_WIDTH']
    parameters['AXI_ADDR_WIDTH'] = 64
    parameters['AXI_STRB_WIDTH'] = parameters['AXI_DATA_WIDTH'] // 8
    parameters['AXI_ID_WIDTH'] = 8
    parameters['AXI_MAX_BURST_LEN'] = 256
    parameters['TLP_FORCE_64_BIT_ADDR'] = 0

    extra_env = {f'PARAM_{k}': str(v) for k, v in parameters.items()}

    extra_env['OFFSET_GROUP'] = str(offset_group)
    extra_env['COCOTB_RESOLVE_X'] = 'RANDOM'

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
