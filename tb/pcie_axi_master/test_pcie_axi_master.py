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
from cocotbext.axi import AxiBus, AxiRam


try:
    from pcie_if import PcieIfDevice, PcieIfRxBus, PcieIfTxBus
except ImportError:
    # attempt import from current directory
    sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
    try:
        from pcie_if import PcieIfDevice, PcieIfRxBus, PcieIfTxBus
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

            rx_req_tlp_bus=PcieIfRxBus.from_prefix(dut, "rx_req_tlp"),

            tx_cpl_tlp_bus=PcieIfTxBus.from_prefix(dut, "tx_cpl_tlp"),

            cfg_max_payload=dut.max_payload_size,
        )

        self.dev.log.setLevel(logging.DEBUG)

        self.dev.functions[0].configure_bar(0, 16*1024*1024)
        self.dev.functions[0].configure_bar(1, 16*1024, io=True)

        self.rc.make_port().connect(self.dev)

        # AXI
        self.axi_ram = AxiRam(AxiBus.from_prefix(dut, "m_axi"), dut.clk, dut.rst, size=2**16)

        # monitor error outputs
        self.status_error_cor_asserted = False
        self.status_error_uncor_asserted = False
        cocotb.start_soon(self._run_monitor_status_error_cor())
        cocotb.start_soon(self._run_monitor_status_error_uncor())

    def set_idle_generator(self, generator=None):
        if generator:
            self.dev.rx_req_tlp_source.set_pause_generator(generator())
            self.axi_ram.write_if.b_channel.set_pause_generator(generator())
            self.axi_ram.read_if.r_channel.set_pause_generator(generator())

    def set_backpressure_generator(self, generator=None):
        if generator:
            self.axi_ram.write_if.aw_channel.set_pause_generator(generator())
            self.axi_ram.write_if.w_channel.set_pause_generator(generator())
            self.axi_ram.read_if.ar_channel.set_pause_generator(generator())

    async def _run_monitor_status_error_cor(self):
        while True:
            await RisingEdge(self.dut.status_error_cor)
            self.log.info("status_error_cor (correctable error) was asserted")
            self.status_error_cor_asserted = True

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

    byte_lanes = tb.axi_ram.write_if.byte_lanes

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

    tb.dut.completer_id.value = int(tb.dev.functions[0].pcie_id)

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

            assert not tb.status_error_cor_asserted
            assert not tb.status_error_uncor_asserted

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_test_read(dut, idle_inserter=None, backpressure_inserter=None):

    tb = TB(dut)

    byte_lanes = tb.axi_ram.read_if.byte_lanes

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

    tb.dut.completer_id.value = int(tb.dev.functions[0].pcie_id)

    for length in list(range(0, byte_lanes*2))+[1024]:
        for pcie_offset in list(range(byte_lanes))+list(range(4096-byte_lanes, 4096)):
            tb.log.info("length %d, pcie_offset %d", length, pcie_offset)
            pcie_addr = pcie_offset+0x1000
            test_data = bytearray([x % 256 for x in range(length)])

            tb.axi_ram.write(pcie_addr-128, b'\x55'*(len(test_data)+256))
            tb.axi_ram.write(pcie_addr, test_data)

            tb.log.debug("%s", tb.axi_ram.hexdump_str((pcie_addr & ~0xf)-16, (((pcie_addr & 0xf)+length-1) & ~0xf)+48, prefix="AXI "))

            val = await dev_bar0.read(pcie_addr, len(test_data), timeout=10000, timeout_unit='ns')

            tb.log.debug("read data: %s", val)

            assert val == test_data

            assert not tb.status_error_cor_asserted
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

    tb.dut.completer_id.value = int(tb.dev.functions[0].pcie_id)

    tb.log.info("Test IO write")

    length = 4
    pcie_addr = 0x1000
    test_data = bytearray([x % 256 for x in range(length)])

    tb.axi_ram.write(pcie_addr-128, b'\x55'*(len(test_data)+256))

    with assert_raises(Exception, "Unsuccessful completion"):
        await dev_bar1.write(pcie_addr, test_data, timeout=1000, timeout_unit='ns')

    await Timer(100, 'ns')

    tb.log.debug("%s", tb.axi_ram.hexdump_str((pcie_addr & ~0xf)-16, (((pcie_addr & 0xf)+length-1) & ~0xf)+48, prefix="AXI "))

    assert tb.axi_ram.read(pcie_addr-1, len(test_data)+2) == b'\x55'*(len(test_data)+2)

    assert tb.status_error_cor_asserted
    assert not tb.status_error_uncor_asserted

    tb.status_error_cor_asserted = False
    tb.status_error_uncor_asserted = False

    tb.log.info("Test IO read")

    length = 4
    pcie_addr = 0x1000
    test_data = bytearray([x % 256 for x in range(length)])

    tb.axi_ram.write(pcie_addr-128, b'\x55'*(len(test_data)+256))
    tb.axi_ram.write(pcie_addr, test_data)

    tb.log.debug("%s", tb.axi_ram.hexdump_str((pcie_addr & ~0xf)-16, (((pcie_addr & 0xf)+length-1) & ~0xf)+48, prefix="AXI "))

    with assert_raises(Exception, "Unsuccessful completion"):
        val = await dev_bar1.read(pcie_addr, len(test_data), timeout=1000, timeout_unit='ns')

    assert tb.status_error_cor_asserted
    assert not tb.status_error_uncor_asserted

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_test_write_aligned_no_extra_cycle(dut, idle_inserter=None, backpressure_inserter=None):
    """Test that aligned writes don't waste extra cycles at the pcie_axi_master level.

    Monitors both the top-level rx_req_tlp signals (input to the demux) and the
    internal write module's signals to pinpoint where any stall originates.

    Only run with no idle/backpressure (the _001 variant) to measure raw performance.
    """
    if idle_inserter is not None or backpressure_inserter is not None:
        return

    tb = TB(dut)

    byte_lanes = tb.axi_ram.write_if.byte_lanes

    await tb.cycle_reset()

    await tb.rc.enumerate()

    dev = tb.rc.find_device(tb.dev.functions[0].pcie_id)
    await dev.enable_device()

    dev_bar0 = dev.bar_window[0]

    tb.dut.completer_id.value = int(tb.dev.functions[0].pcie_id)

    for length in [byte_lanes * 2, byte_lanes * 4, 256]:
        if length < byte_lanes * 2:
            continue

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

        tb.log.info("Aligned write %dB: %d TLPs, %d input beats, %d stall cycles",
                     length, stall_info['tlp_count'], stall_info['total_input_beats'],
                     stall_info['stall_cycles'])

        assert stall_info['stall_cycles'] == 0, \
            f"Aligned write of {length}B had {stall_info['stall_cycles']} stall cycle(s) at pcie_axi_master level!"

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_test_back_to_back_writes(dut, idle_inserter=None, backpressure_inserter=None):
    """Test back-to-back write TLPs with various sizes.

    Exercises the combinational demux frame tracking by sending multiple writes
    in quick succession, ensuring the select register resets correctly between
    TLPs and data is routed without corruption.
    """
    tb = TB(dut)

    byte_lanes = tb.axi_ram.write_if.byte_lanes

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    await tb.cycle_reset()

    await tb.rc.enumerate()

    dev = tb.rc.find_device(tb.dev.functions[0].pcie_id)
    await dev.enable_device()

    dev_bar0 = dev.bar_window[0]

    tb.dut.completer_id.value = int(tb.dev.functions[0].pcie_id)

    # Rapid-fire writes at different addresses and sizes
    write_configs = [
        (0x1000, 4),               # single DW
        (0x2000, byte_lanes),      # exactly one beat
        (0x3000, byte_lanes * 2),  # two beats
        (0x4000, byte_lanes * 4),  # four beats
        (0x5000, 1),               # single byte
        (0x6000, 3),               # sub-DW
        (0x7000, byte_lanes + 1),  # one beat + 1 byte overflow
    ]

    # Issue all writes back-to-back without waiting
    expected = {}
    for pcie_addr, length in write_configs:
        test_data = bytearray([(x + pcie_addr) % 256 for x in range(length)])
        tb.axi_ram.write(pcie_addr - 128, b'\x55' * (length + 256))
        expected[pcie_addr] = test_data
        await dev_bar0.write(pcie_addr, test_data)

    # Wait long enough for all writes to complete
    await Timer(3000, 'ns')

    # Verify all writes landed correctly
    for pcie_addr, length in write_configs:
        test_data = expected[pcie_addr]
        tb.log.info("Verifying back-to-back write: addr=0x%04x, len=%d", pcie_addr, length)
        assert tb.axi_ram.read(pcie_addr - 1, len(test_data) + 2) == b'\x55' + test_data + b'\x55', \
            f"Data mismatch at addr=0x{pcie_addr:04x}, len={length}"

    assert not tb.status_error_cor_asserted
    assert not tb.status_error_uncor_asserted

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_test_back_to_back_reads(dut, idle_inserter=None, backpressure_inserter=None):
    """Test back-to-back read TLPs with various sizes.

    Ensures the combinational demux routes consecutive reads correctly to the
    read sub-module without frame tracking errors.
    """
    tb = TB(dut)

    byte_lanes = tb.axi_ram.read_if.byte_lanes

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    await tb.cycle_reset()

    await tb.rc.enumerate()

    dev = tb.rc.find_device(tb.dev.functions[0].pcie_id)
    await dev.enable_device()

    dev_bar0 = dev.bar_window[0]

    tb.dut.completer_id.value = int(tb.dev.functions[0].pcie_id)

    read_configs = [
        (0x1000, 4),               # single DW
        (0x2000, byte_lanes),      # one beat
        (0x3000, byte_lanes * 2),  # two beats
        (0x4000, 1),               # single byte
        (0x5000, byte_lanes * 4),  # four beats
    ]

    # Prepare data in RAM
    for pcie_addr, length in read_configs:
        test_data = bytearray([(x + pcie_addr) % 256 for x in range(length)])
        tb.axi_ram.write(pcie_addr, test_data)

    # Perform reads back-to-back
    for pcie_addr, length in read_configs:
        expected = bytearray([(x + pcie_addr) % 256 for x in range(length)])
        tb.log.info("Reading back-to-back: addr=0x%04x, len=%d", pcie_addr, length)
        val = await dev_bar0.read(pcie_addr, length, timeout=10000, timeout_unit='ns')
        assert val == expected, \
            f"Read data mismatch at addr=0x{pcie_addr:04x}, len={length}"

    assert not tb.status_error_cor_asserted
    assert not tb.status_error_uncor_asserted

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_test_write_then_read(dut, idle_inserter=None, backpressure_inserter=None):
    """Test a write followed by a read to the same address.

    Exercises the combinational demux switching from write port to read port,
    verifying the frame tracking register correctly releases after the write TLP
    completes and the read TLP is routed correctly.
    """
    tb = TB(dut)

    byte_lanes = tb.axi_ram.write_if.byte_lanes

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    await tb.cycle_reset()

    await tb.rc.enumerate()

    dev = tb.rc.find_device(tb.dev.functions[0].pcie_id)
    await dev.enable_device()

    dev_bar0 = dev.bar_window[0]

    tb.dut.completer_id.value = int(tb.dev.functions[0].pcie_id)

    for length in [1, 4, byte_lanes, byte_lanes * 2, byte_lanes * 4]:
        pcie_addr = 0x1000
        test_data = bytearray([(x + length) % 256 for x in range(length)])

        tb.axi_ram.write(pcie_addr - 128, b'\x55' * (length + 256))

        tb.log.info("Write-then-read: len=%d", length)

        # Write data
        await dev_bar0.write(pcie_addr, test_data)
        await Timer(length * 4 + 200, 'ns')

        # Verify write
        assert tb.axi_ram.read(pcie_addr, length) == test_data

        # Read it back via PCIe (tests demux switching from write->read)
        val = await dev_bar0.read(pcie_addr, length, timeout=10000, timeout_unit='ns')
        assert val == test_data, \
            f"Write-then-read mismatch at len={length}"

    assert not tb.status_error_cor_asserted
    assert not tb.status_error_uncor_asserted

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_test_read_then_write(dut, idle_inserter=None, backpressure_inserter=None):
    """Test a read followed by a write.

    Exercises demux switching from read port to write port, verifying the
    select logic correctly identifies the write TLP type after handling reads.
    """
    tb = TB(dut)

    byte_lanes = tb.axi_ram.write_if.byte_lanes

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    await tb.cycle_reset()

    await tb.rc.enumerate()

    dev = tb.rc.find_device(tb.dev.functions[0].pcie_id)
    await dev.enable_device()

    dev_bar0 = dev.bar_window[0]

    tb.dut.completer_id.value = int(tb.dev.functions[0].pcie_id)

    for length in [1, 4, byte_lanes, byte_lanes * 2]:
        pcie_addr = 0x2000

        # Pre-fill with known data for read
        read_data = bytearray([(x * 3 + length) % 256 for x in range(length)])
        tb.axi_ram.write(pcie_addr, read_data)

        tb.log.info("Read-then-write: len=%d", length)

        # Read first (demux selects read port)
        val = await dev_bar0.read(pcie_addr, length, timeout=10000, timeout_unit='ns')
        assert val == read_data, \
            f"Read mismatch at len={length}"

        # Now write different data (demux must switch to write port)
        write_data = bytearray([(x * 7 + length) % 256 for x in range(length)])
        tb.axi_ram.write(pcie_addr - 1, b'\x55' * (length + 2))
        await dev_bar0.write(pcie_addr, write_data)
        await Timer(length * 4 + 200, 'ns')

        assert tb.axi_ram.read(pcie_addr - 1, length + 2) == b'\x55' + write_data + b'\x55', \
            f"Write mismatch after read at len={length}"

    assert not tb.status_error_cor_asserted
    assert not tb.status_error_uncor_asserted

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_test_write_single_dword(dut, idle_inserter=None, backpressure_inserter=None):
    """Test single-DWORD writes (1-4 bytes, single-beat AXI, awlen==0 path).

    These exercise the fallback path in pcie_axi_master_wr where first_cycle is
    used instead of the multi-beat optimization.
    """
    tb = TB(dut)

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    await tb.cycle_reset()

    await tb.rc.enumerate()

    dev = tb.rc.find_device(tb.dev.functions[0].pcie_id)
    await dev.enable_device()

    dev_bar0 = dev.bar_window[0]

    tb.dut.completer_id.value = int(tb.dev.functions[0].pcie_id)

    # Single-DWORD writes at various sub-DWORD lengths and offsets
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

    assert not tb.status_error_cor_asserted
    assert not tb.status_error_uncor_asserted

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_test_large_write(dut, idle_inserter=None, backpressure_inserter=None):
    """Test large writes that span multiple TLPs.

    These exercise multi-TLP frame tracking and the burst continuation logic
    in pcie_axi_master_wr, as a 1024-byte write will be split into multiple
    TLPs by the PCIe max payload size (128B default).
    """
    tb = TB(dut)

    byte_lanes = tb.axi_ram.write_if.byte_lanes

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    await tb.cycle_reset()

    await tb.rc.enumerate()

    dev = tb.rc.find_device(tb.dev.functions[0].pcie_id)
    await dev.enable_device()

    dev_bar0 = dev.bar_window[0]

    tb.dut.completer_id.value = int(tb.dev.functions[0].pcie_id)

    for length in [128, 256, 512, 1024]:
        pcie_addr = 0x1000
        test_data = bytearray([x % 256 for x in range(length)])

        tb.axi_ram.write(pcie_addr - 128, b'\x55' * (length + 256))

        tb.log.info("Large write: len=%d", length)
        await dev_bar0.write(pcie_addr, test_data)
        await Timer(length * 4 + 500, 'ns')

        assert tb.axi_ram.read(pcie_addr - 1, length + 2) == b'\x55' + test_data + b'\x55', \
            f"Large write data mismatch at len={length}"

    assert not tb.status_error_cor_asserted
    assert not tb.status_error_uncor_asserted

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_test_interleaved_operations(dut, idle_inserter=None, backpressure_inserter=None):
    """Test interleaved reads and writes at the combined pcie_axi_master level.

    Rapidly alternates between read and write operations to stress the
    combinational demux port switching logic.
    """
    tb = TB(dut)

    byte_lanes = tb.axi_ram.write_if.byte_lanes

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    await tb.cycle_reset()

    await tb.rc.enumerate()

    dev = tb.rc.find_device(tb.dev.functions[0].pcie_id)
    await dev.enable_device()

    dev_bar0 = dev.bar_window[0]

    tb.dut.completer_id.value = int(tb.dev.functions[0].pcie_id)

    # Interleave: write A, read B, write C, read A, write B, read C
    addrs = [0x1000, 0x2000, 0x3000]
    length = byte_lanes * 2

    # Initialize all regions
    for addr in addrs:
        tb.axi_ram.write(addr - 128, b'\x55' * (length + 256))

    # Round 1: write to A, then read from uninitialized B (expect 0x55)
    data_a = bytearray([(x + 0xA0) % 256 for x in range(length)])
    tb.log.info("Interleaved: write A")
    await dev_bar0.write(addrs[0], data_a)
    await Timer(length * 4 + 200, 'ns')

    tb.log.info("Interleaved: read B (expect 0x55)")
    val = await dev_bar0.read(addrs[1], length, timeout=10000, timeout_unit='ns')
    assert val == b'\x55' * length

    # Round 2: write to B, then read back A
    data_b = bytearray([(x + 0xB0) % 256 for x in range(length)])
    tb.log.info("Interleaved: write B")
    await dev_bar0.write(addrs[1], data_b)
    await Timer(length * 4 + 200, 'ns')

    tb.log.info("Interleaved: read A")
    val = await dev_bar0.read(addrs[0], length, timeout=10000, timeout_unit='ns')
    assert val == data_a, "Read-back of A after write to B failed"

    # Round 3: write to C, then read back B
    data_c = bytearray([(x + 0xC0) % 256 for x in range(length)])
    tb.log.info("Interleaved: write C")
    await dev_bar0.write(addrs[2], data_c)
    await Timer(length * 4 + 200, 'ns')

    tb.log.info("Interleaved: read B")
    val = await dev_bar0.read(addrs[1], length, timeout=10000, timeout_unit='ns')
    assert val == data_b, "Read-back of B after write to C failed"

    # Final: read C
    tb.log.info("Interleaved: read C")
    val = await dev_bar0.read(addrs[2], length, timeout=10000, timeout_unit='ns')
    assert val == data_c, "Read-back of C failed"

    assert not tb.status_error_cor_asserted
    assert not tb.status_error_uncor_asserted

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_test_unaligned_write_no_stall(dut, idle_inserter=None, backpressure_inserter=None):
    """Test that unaligned multi-beat writes also don't stall.

    The aligned no_extra_cycle test only checks aligned addresses. This test
    verifies that the first-beat optimization works for unaligned writes too,
    where the first beat has a partial strobe mask.

    Only run without idle/backpressure to measure raw performance.
    """
    if idle_inserter is not None or backpressure_inserter is not None:
        return

    tb = TB(dut)

    byte_lanes = tb.axi_ram.write_if.byte_lanes

    await tb.cycle_reset()

    await tb.rc.enumerate()

    dev = tb.rc.find_device(tb.dev.functions[0].pcie_id)
    await dev.enable_device()

    dev_bar0 = dev.bar_window[0]

    tb.dut.completer_id.value = int(tb.dev.functions[0].pcie_id)

    for offset in [1, 2, byte_lanes // 2, byte_lanes - 1]:
        for length in [byte_lanes * 2, byte_lanes * 4]:
            pcie_addr = 0x1000 + offset
            test_data = bytearray([(x + offset) % 256 for x in range(length)])

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

            tb.log.info("Unaligned write offset=%d len=%dB: %d TLPs, %d input beats, %d stall cycles",
                         offset, length, stall_info['tlp_count'], stall_info['total_input_beats'],
                         stall_info['stall_cycles'])

            assert stall_info['stall_cycles'] == 0, \
                f"Unaligned write offset={offset} len={length}B had {stall_info['stall_cycles']} stall cycle(s)!"

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_test_write_stress(dut, idle_inserter=None, backpressure_inserter=None):
    """Stress test: many random writes with various sizes and offsets.

    Exercises the full write path (combinational demux + first-beat optimization)
    under heavy, varied traffic patterns to catch rare edge cases.
    """
    tb = TB(dut)

    byte_lanes = tb.axi_ram.write_if.byte_lanes

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    await tb.cycle_reset()

    await tb.rc.enumerate()

    dev = tb.rc.find_device(tb.dev.functions[0].pcie_id)
    await dev.enable_device()

    dev_bar0 = dev.bar_window[0]

    tb.dut.completer_id.value = int(tb.dev.functions[0].pcie_id)

    random.seed(123)

    write_ops = []
    for i in range(30):
        length = random.randint(1, 128)
        offset = random.randint(0, byte_lanes - 1)
        pcie_addr = 0x200 + i * 0x400 + offset
        test_data = bytearray([random.randint(0, 255) for _ in range(length)])

        tb.axi_ram.write(pcie_addr - 128, b'\x55' * (length + 256))
        write_ops.append((pcie_addr, test_data))

        await dev_bar0.write(pcie_addr, test_data)

    await Timer(10000, 'ns')

    for pcie_addr, test_data in write_ops:
        length = len(test_data)
        assert tb.axi_ram.read(pcie_addr - 1, length + 2) == b'\x55' + test_data + b'\x55', \
            f"Stress write mismatch at addr=0x{pcie_addr:04x}, len={length}"

    assert not tb.status_error_cor_asserted
    assert not tb.status_error_uncor_asserted

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def run_test_mixed_stress(dut, idle_inserter=None, backpressure_inserter=None):
    """Stress test: random mix of reads and writes.

    Exercises the combinational demux port switching under heavy mixed traffic,
    verifying that frame tracking correctly handles rapid alternation between
    write and read TLP types.
    """
    tb = TB(dut)

    byte_lanes = tb.axi_ram.write_if.byte_lanes

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    await tb.cycle_reset()

    await tb.rc.enumerate()

    dev = tb.rc.find_device(tb.dev.functions[0].pcie_id)
    await dev.enable_device()

    dev_bar0 = dev.bar_window[0]

    tb.dut.completer_id.value = int(tb.dev.functions[0].pcie_id)

    random.seed(456)

    # Phase 1: Write random data to various addresses
    write_ops = []
    for i in range(20):
        length = random.randint(1, 128)
        pcie_addr = 0x1000 + i * 0x200
        test_data = bytearray([random.randint(0, 255) for _ in range(length)])

        tb.axi_ram.write(pcie_addr - 128, b'\x55' * (length + 256))
        write_ops.append((pcie_addr, test_data))
        await dev_bar0.write(pcie_addr, test_data)

    await Timer(5000, 'ns')

    # Phase 2: Verify writes, then read them back via PCIe
    for pcie_addr, test_data in write_ops:
        length = len(test_data)
        # Verify via direct RAM check
        assert tb.axi_ram.read(pcie_addr - 1, length + 2) == b'\x55' + test_data + b'\x55'
        # Verify via PCIe read
        val = await dev_bar0.read(pcie_addr, length, timeout=10000, timeout_unit='ns')
        assert val == test_data, \
            f"Mixed stress read-back mismatch at addr=0x{pcie_addr:04x}, len={length}"

    assert not tb.status_error_cor_asserted
    assert not tb.status_error_uncor_asserted

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


def cycle_pause():
    return itertools.cycle([1, 1, 1, 0])


if cocotb.SIM_NAME:

    for test in [
                run_test_write,
                run_test_read,
                run_test_bad_ops,
                run_test_write_aligned_no_extra_cycle,
                run_test_back_to_back_writes,
                run_test_back_to_back_reads,
                run_test_write_then_read,
                run_test_read_then_write,
                run_test_write_single_dword,
                run_test_large_write,
                run_test_interleaved_operations,
                run_test_unaligned_write_no_stall,
                run_test_write_stress,
                run_test_mixed_stress,
            ]:

        factory = TestFactory(test)
        factory.add_option(("idle_inserter", "backpressure_inserter"), [(None, None), (cycle_pause, cycle_pause)])
        factory.generate_tests()


# cocotb-test

tests_dir = os.path.dirname(__file__)
rtl_dir = os.path.abspath(os.path.join(tests_dir, '..', '..', 'rtl'))


@pytest.mark.parametrize("offset_group", list(range(8)))
@pytest.mark.parametrize("pcie_data_width", [64, 128])
def test_pcie_axi_master(request, pcie_data_width, offset_group):
    dut = "pcie_axi_master"
    module = os.path.splitext(os.path.basename(__file__))[0]
    toplevel = dut

    verilog_sources = [
        os.path.join(rtl_dir, f"{dut}.v"),
        os.path.join(rtl_dir, f"{dut}_rd.v"),
        os.path.join(rtl_dir, f"{dut}_wr.v"),
        os.path.join(rtl_dir, "pulse_merge.v"),
    ]

    parameters = {}

    parameters['TLP_DATA_WIDTH'] = pcie_data_width
    parameters['TLP_STRB_WIDTH'] = parameters['TLP_DATA_WIDTH'] // 32
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
