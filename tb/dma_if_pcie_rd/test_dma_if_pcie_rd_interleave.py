#!/usr/bin/env python
"""
Stress test for dma_if_pcie_rd: many concurrent small reads to two completers
with very different latencies ("peer" = short latency, one completion per
request; "host" = long latency, completions split at 64 B RCB boundaries).
Completions for different tags are interleaved TLP by TLP in random order.
The test drives the TLP completion interface directly (no cocotbext-pcie RC
model) and implements its own segmented RAM model with delayed, in-order
per-segment write-done pulses.  At every read status the RAM contents of that
operation (plus guard bytes) are checked.

Knobs (environment variables):
  SEED, N_OPS, MAX_OUTSTANDING, PEER_LENS, HOST_LENS, PEER_LAT, HOST_LAT,
  PEER_SPLIT (0 = one completion, else split size), HOST_RCB, HOST_MERGE
  (max RCB chunks merged in one completion), RX_IDLE, RAM_BP, REQ_BP,
  RAM_DONE_LAT, SAME_SEL, BAD_INTERLEAVE (illegal: start another TLP between
  beats of a multi-beat TLP; used only to characterize the engine)
"""

import collections
import logging
import os
import random

import cocotb_test.simulator

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Event
from cocotbext.axi.stream import define_stream

DescBus, DescTransaction, DescSource, DescSink, DescMonitor = define_stream("Desc",
    signals=["pcie_addr", "ram_addr", "ram_sel", "len", "tag", "valid", "ready"]
)

DescStatusBus, DescStatusTransaction, DescStatusSource, DescStatusSink, DescStatusMonitor = define_stream("DescStatus",
    signals=["tag", "error", "valid"]
)

HOST_BASE = 0x0000_0001_2340_0000
PEER_BASE = 0x0000_0038_0000_0000
MEM_SIZE = 1 << 20
SLOT_SIZE = 512
RAM_SLOT_BASE = 0x1000


def env_int(name, default):
    v = os.getenv(name)
    return int(v, 0) if v else default


def env_list(name, default):
    v = os.getenv(name)
    if not v:
        return default
    return [int(x, 0) for x in v.split(",") if x]


def env_range(name, default):
    v = env_list(name, default)
    return (v[0], v[-1])


def lowest_bit(x):
    for k in range(4):
        if x & (1 << k):
            return k
    return 0


def highest_bit(x):
    for k in range(3, -1, -1):
        if x & (1 << k):
            return k
    return 0


class Cpl:
    def __init__(self, tag, hdr, beats, arrival, info, kind="", final=True):
        self.kind = kind
        self.final = final
        self.tag = tag
        self.hdr = hdr
        self.beats = beats
        self.arrival = arrival
        self.info = info


class TB:
    def __init__(self, dut):
        self.dut = dut
        self.log = logging.getLogger("cocotb.tb")
        self.log.setLevel(logging.INFO)

        self.seed = env_int("SEED", 1)
        self.rng = random.Random(self.seed)

        self.tlp_bytes = len(dut.rx_cpl_tlp_data) // 8
        self.seg_count = len(dut.ram_wr_cmd_valid)
        self.seg_bytes = len(dut.ram_wr_cmd_be) // self.seg_count
        self.seg_addr_width = len(dut.ram_wr_cmd_addr) // self.seg_count
        self.sel_width = len(dut.ram_wr_cmd_sel) // self.seg_count
        self.tag_width = len(dut.s_axis_read_desc_tag)

        self.peer_lat = env_range("PEER_LAT", [20, 60])
        self.host_lat = env_range("HOST_LAT", [150, 400])
        self.peer_split = env_int("PEER_SPLIT", 0)
        self.host_rcb = env_int("HOST_RCB", 64)
        self.host_merge = env_int("HOST_MERGE", 1)
        self.rx_idle = env_int("RX_IDLE", 20) / 100.0
        self.ram_bp = env_int("RAM_BP", 20) / 100.0
        self.req_bp = env_int("REQ_BP", 10) / 100.0
        self.ram_done_lat = env_range("RAM_DONE_LAT", [1, 8])
        self.bad_interleave = env_int("BAD_INTERLEAVE", 0)

        cocotb.start_soon(Clock(dut.clk, 4, units="ns").start())

        self.read_desc_source = DescSource(DescBus.from_prefix(dut, "s_axis_read_desc"), dut.clk, dut.rst)
        self.read_desc_status_sink = DescStatusSink(DescStatusBus.from_prefix(dut, "m_axis_read_desc_status"), dut.clk, dut.rst)

        self.host_mem = bytearray(MEM_SIZE)
        self.peer_mem = bytearray(MEM_SIZE)
        self.ram = [bytearray(1 << len(dut.s_axis_read_desc_ram_addr)) for _ in range(1 << self.sel_width)]

        self.cycle = 0
        self.cpl_queues = collections.OrderedDict()  # pcie tag -> deque of Cpl
        self.req_count = 0
        self.cpl_tlp_count = 0
        self.interleave_events = 0
        self.partial = {}
        self.split_interleave = 0

        dut.rx_cpl_tlp_data.setimmediatevalue(0)
        dut.rx_cpl_tlp_hdr.setimmediatevalue(0)
        dut.rx_cpl_tlp_error.setimmediatevalue(0)
        dut.rx_cpl_tlp_valid.setimmediatevalue(0)
        dut.rx_cpl_tlp_sop.setimmediatevalue(0)
        dut.rx_cpl_tlp_eop.setimmediatevalue(0)
        dut.tx_rd_req_tlp_ready.setimmediatevalue(0)
        dut.s_axis_tx_seq_num.setimmediatevalue(0)
        dut.s_axis_tx_seq_num_valid.setimmediatevalue(0)
        dut.ram_wr_cmd_ready.setimmediatevalue(0)
        dut.ram_wr_done.setimmediatevalue(0)
        dut.enable.setimmediatevalue(0)
        dut.ext_tag_enable.setimmediatevalue(1)
        dut.rcb_128b.setimmediatevalue(0)
        dut.requester_id.setimmediatevalue(0x0100)
        dut.max_read_request_size.setimmediatevalue(2)

        self.cor_err = 0
        self.uncor_err = 0

    async def reset(self):
        self.dut.rst.setimmediatevalue(0)
        for _ in range(2):
            await RisingEdge(self.dut.clk)
        self.dut.rst.value = 1
        for _ in range(2):
            await RisingEdge(self.dut.clk)
        self.dut.rst.value = 0
        for _ in range(2):
            await RisingEdge(self.dut.clk)

    def start(self):
        cocotb.start_soon(self._run_cycle())
        cocotb.start_soon(self._run_req())
        cocotb.start_soon(self._run_rx())
        cocotb.start_soon(self._run_ram())
        cocotb.start_soon(self._run_err())

    async def _run_cycle(self):
        while True:
            await RisingEdge(self.dut.clk)
            self.cycle += 1

    async def _run_err(self):
        while True:
            await RisingEdge(self.dut.clk)
            if self.dut.status_error_cor.value:
                self.cor_err += 1
            if self.dut.status_error_uncor.value:
                self.uncor_err += 1

    # ---------------------------------------------------------------- requests
    async def _run_req(self):
        dut = self.dut
        while True:
            await RisingEdge(dut.clk)
            dut.s_axis_tx_seq_num_valid.value = 0
            if dut.tx_rd_req_tlp_valid.value and dut.tx_rd_req_tlp_ready.value:
                hdr = dut.tx_rd_req_tlp_hdr.value.integer
                self.handle_req(hdr)
                dut.s_axis_tx_seq_num_valid.value = 1
            dut.tx_rd_req_tlp_ready.value = 0 if self.rng.random() < self.req_bp else 1

    def handle_req(self, hdr):
        fmt = (hdr >> 125) & 7
        length = (hdr >> 96) & 0x3ff or 1024
        tag = ((hdr >> 72) & 0xff) | (((hdr >> 115) & 1) << 8) | (((hdr >> 119) & 1) << 9)
        req_id = (hdr >> 80) & 0xffff
        last_be = (hdr >> 68) & 0xf
        first_be = (hdr >> 64) & 0xf
        if fmt & 1:
            addr = ((hdr >> 2) & ((1 << 62) - 1)) << 2
        else:
            addr = ((hdr >> 34) & ((1 << 30) - 1)) << 2

        if length == 1:
            start = addr + lowest_bit(first_be)
            end = addr + highest_bit(first_be) + 1
        else:
            start = addr + lowest_bit(first_be)
            end = addr + (length - 1) * 4 + highest_bit(last_be) + 1

        if HOST_BASE <= addr < HOST_BASE + MEM_SIZE:
            mem, base, kind = self.host_mem, HOST_BASE, "host"
            lat = self.rng.randint(*self.host_lat)
            boundaries = self.host_rcb
            merge = self.host_merge
        elif PEER_BASE <= addr < PEER_BASE + MEM_SIZE:
            mem, base, kind = self.peer_mem, PEER_BASE, "peer"
            lat = self.rng.randint(*self.peer_lat)
            boundaries = self.peer_split
            merge = 1
        else:
            raise Exception(f"request to unknown address {addr:#x}")

        assert tag not in self.cpl_queues or not self.cpl_queues[tag], f"tag {tag} reused while outstanding"

        # split into completions
        chunks = []
        cur = start
        while cur < end:
            if boundaries:
                n = self.rng.randint(1, merge)
                cend = min(((cur // boundaries) + n) * boundaries, end)
            else:
                cend = end
            chunks.append((cur, cend))
            cur = cend

        q = self.cpl_queues.setdefault(tag, collections.deque())
        t = self.cycle + lat
        for cstart, cend in chunks:
            byte_count = end - cstart
            dw_start = cstart & ~3
            dw_end = (cend + 3) & ~3
            length_dw = (dw_end - dw_start) // 4
            payload = bytes(mem[dw_start - base:dw_end - base])
            h = 0
            h |= 0b010 << 125
            h |= 0b01010 << 120
            h |= ((tag >> 9) & 1) << 119
            h |= ((tag >> 8) & 1) << 115
            h |= (length_dw & 0x3ff) << 96
            h |= 0x0000 << 80
            h |= (byte_count & 0xfff) << 64
            h |= req_id << 48
            h |= (tag & 0xff) << 40
            h |= (cstart & 0x7f) << 32
            beats = []
            for k in range(0, len(payload), self.tlp_bytes):
                beats.append(int.from_bytes(payload[k:k + self.tlp_bytes], 'little'))
            q.append(Cpl(tag, h, beats, t, f"{kind} tag {tag} [{cstart - base:#x},{cend - base:#x}) bc {byte_count}",
                         kind=kind, final=(cend == end)))
            t += self.rng.randint(1, 6)
        self.req_count += 1

    # ------------------------------------------------------------- completions
    def pick_cpl(self, exclude_tag=None, filt=None):
        avail = [tag for tag, q in self.cpl_queues.items() if q and q[0].arrival <= self.cycle and tag != exclude_tag
                 and (filt is None or filt(q[0]))]
        if not avail:
            return None
        tag = self.rng.choice(avail)
        return self.cpl_queues[tag].popleft()

    async def _run_rx(self):
        dut = self.dut
        cur = None
        beat = 0
        stack = []
        presented = False
        while True:
            await RisingEdge(dut.clk)
            if presented and dut.rx_cpl_tlp_ready.value:
                beat += 1
                presented = False
                if beat >= len(cur.beats):
                    self.cpl_tlp_count += 1
                    if stack:
                        cur, beat = stack.pop()
                    else:
                        cur = None
                elif self.bad_interleave and not stack and self.rng.random() < 0.5:
                    # 1: any TLP; 2: host TLP into peer TLP; 3: host non-final TLP into peer TLP
                    filt = None
                    if self.bad_interleave == 2:
                        filt = lambda c: c.kind == "host"
                    elif self.bad_interleave == 3:
                        filt = lambda c: c.kind == "host" and not c.final
                    other = None
                    if self.bad_interleave == 1 or cur.kind == "peer":
                        other = self.pick_cpl(exclude_tag=cur.tag, filt=filt)
                    if other is not None:
                        self.interleave_events += 1
                        stack.append((cur, beat))
                        cur, beat = other, 0

            if presented:
                continue

            if cur is None:
                cur = self.pick_cpl()
                beat = 0
                if cur is not None:
                    # count TLPs that start while another tag has delivered only part of its completions
                    if any(t != cur.tag and self.partial.get(t) and self.cpl_queues[t] for t in self.partial):
                        self.split_interleave += 1
                    if self.cpl_queues[cur.tag]:
                        self.partial[cur.tag] = True
                    else:
                        self.partial.pop(cur.tag, None)

            if cur is None or self.rng.random() < self.rx_idle:
                dut.rx_cpl_tlp_valid.value = 0
                continue

            dut.rx_cpl_tlp_hdr.value = cur.hdr
            dut.rx_cpl_tlp_data.value = cur.beats[beat]
            dut.rx_cpl_tlp_sop.value = 1 if beat == 0 else 0
            dut.rx_cpl_tlp_eop.value = 1 if beat == len(cur.beats) - 1 else 0
            dut.rx_cpl_tlp_valid.value = 1
            presented = True

    # --------------------------------------------------------------------- RAM
    async def _run_ram(self):
        dut = self.dut
        sc = self.seg_count
        sb = self.seg_bytes
        done_q = [collections.deque() for _ in range(sc)]
        while True:
            await RisingEdge(dut.clk)
            valid = dut.ram_wr_cmd_valid.value.integer
            ready = dut.ram_wr_cmd_ready.value.integer
            if valid & ready:
                be_all = dut.ram_wr_cmd_be.value.integer
                addr_all = dut.ram_wr_cmd_addr.value.integer
                data_all = dut.ram_wr_cmd_data.value.integer
                sel_all = dut.ram_wr_cmd_sel.value.integer
                for n in range(sc):
                    if not (valid & ready) & (1 << n):
                        continue
                    be = (be_all >> (n * sb)) & ((1 << sb) - 1)
                    seg_addr = (addr_all >> (n * self.seg_addr_width)) & ((1 << self.seg_addr_width) - 1)
                    data = ((data_all >> (n * sb * 8)) & ((1 << (sb * 8)) - 1)).to_bytes(sb, 'little')
                    sel = (sel_all >> (n * self.sel_width)) & ((1 << self.sel_width) - 1)
                    base = (seg_addr * sc + n) * sb
                    ram = self.ram[sel]
                    for i in range(sb):
                        if be & (1 << i):
                            ram[base + i] = data[i]
                    t = self.cycle + self.rng.randint(*self.ram_done_lat)
                    if done_q[n]:
                        t = max(t, done_q[n][-1])
                    done_q[n].append(t)
            done = 0
            for n in range(sc):
                if done_q[n] and done_q[n][0] <= self.cycle:
                    done_q[n].popleft()
                    done |= 1 << n
            dut.ram_wr_done.value = done
            r = 0
            for n in range(sc):
                if self.rng.random() >= self.ram_bp:
                    r |= 1 << n
            dut.ram_wr_cmd_ready.value = r


async def run_test_interleave(dut):
    tb = TB(dut)
    rng = tb.rng

    n_ops = env_int("N_OPS", 2000)
    max_out = env_int("MAX_OUTSTANDING", 64)
    peer_lens = env_list("PEER_LENS", [96])
    host_lens = env_list("HOST_LENS", [68])
    same_sel = env_int("SAME_SEL", 0)
    n_slots = 2 * max_out + 8

    await tb.reset()
    tb.start()
    dut.enable.value = 1

    for _ in range(300):
        await RisingEdge(dut.clk)

    free_slots = list(range(n_slots))
    rng.shuffle(free_slots)
    ops = {}
    slot_free_event = Event()
    errors = []
    stats = collections.Counter()

    def describe_bytes(got, exp, op):
        # try to find where corrupted data came from
        out = []
        for i in range(len(exp)):
            if got[i] != exp[i]:
                out.append(i)
        return out

    async def status_loop():
        while True:
            st = await tb.read_desc_status_sink.recv()
            tag = int(st.tag)
            err = int(st.error)
            op = ops.pop(tag)
            ram = tb.ram[op["sel"]]
            ra = op["ram_addr"]
            ln = op["len"]
            got = bytes(ram[ra - 16:ra + ln + 16])
            exp = b'\xaa' * 16 + op["data"] + b'\xaa' * 16
            stats[op["kind"]] += 1
            if err != 0 or got != exp:
                bad = [i - 16 for i in range(len(exp)) if got[i] != exp[i]]
                stats[op["kind"] + "_bad"] += 1
                msg = (f"op {op['id']} {op['kind']} tag {tag} err {err} len {ln} src_off {op['src'] & 0x7f:#x} "
                       f"ram {ra:#x} sel {op['sel']} bad bytes {bad[:4]}..{bad[-4:]} (n={len(bad)})")
                if bad:
                    b0 = bad[0]
                    frag = got[16 + b0:16 + min(bad[-1] + 1, b0 + 16)]
                    origin = "stale (0xaa)" if all(x == 0xaa for x in frag) else "unknown"
                    if origin == "unknown":
                        for m, mname in ((tb.host_mem, "host_mem"), (tb.peer_mem, "peer_mem")):
                            k = m.find(frag)
                            if k >= 0:
                                origin = f"{mname}+{k:#x} (slot {k // SLOT_SIZE}, off {k % SLOT_SIZE:#x})"
                                break
                    msg += f"; corrupted bytes from {origin}"
                tb.log.error(msg)
                errors.append(msg)
            free_slots.append(op["slot"])
            slot_free_event.set()

    cocotb.start_soon(status_loop())

    for i in range(n_ops):
        while not free_slots or len(ops) >= max_out:
            slot_free_event.clear()
            await slot_free_event.wait()

        slot = free_slots.pop(0)
        kind = "peer" if rng.random() < 0.5 else "host"
        if kind == "peer":
            ln = rng.choice(peer_lens)
            mem, base = tb.peer_mem, PEER_BASE
            sel = 0
        else:
            ln = rng.choice(host_lens)
            mem, base = tb.host_mem, HOST_BASE
            sel = 0 if same_sel else 1

        src_off = rng.choice([0, 0, 4, 8, 32, 60, 64, rng.randrange(0, 128), rng.randrange(0, 128) & ~3])
        src = slot * SLOT_SIZE + src_off
        data = bytes(rng.getrandbits(8) for _ in range(ln))
        mem[src:src + ln] = data

        ram_off = rng.choice([0, 0, 1, 4, 32, 63, 64, 96, 127, rng.randrange(0, 128)])
        ram_addr = RAM_SLOT_BASE + slot * SLOT_SIZE + ram_off
        tb.ram[sel][ram_addr - 16:ram_addr + ln + 16] = b'\xaa' * (ln + 32)

        tag = i % (1 << tb.tag_width)
        ops[tag] = dict(id=i, kind=kind, len=ln, src=src, ram_addr=ram_addr, sel=sel, slot=slot, data=data)

        desc = DescTransaction(pcie_addr=base + src, ram_addr=ram_addr, ram_sel=sel, len=ln, tag=tag)
        await tb.read_desc_source.send(desc)

        if i % 500 == 0:
            tb.log.info("issued %d ops, cycle %d, errors %d", i, tb.cycle, len(errors))

    timeout = 0
    while ops and timeout < 200000:
        await RisingEdge(dut.clk)
        timeout += 1

    tb.log.info("stats: %s, requests %d, cpl TLPs %d, cycles %d, cor_err %d, uncor_err %d, split_interleave %d, bad_interleave_events %d",
                dict(stats), tb.req_count, tb.cpl_tlp_count, tb.cycle, tb.cor_err, tb.uncor_err, tb.split_interleave, tb.interleave_events)

    assert not ops, f"{len(ops)} ops never completed"
    assert not errors, f"{len(errors)} corrupted ops; first: {errors[0]}"
    assert tb.cor_err == 0 and tb.uncor_err == 0


if cocotb.SIM_NAME:
    run_test_interleave = cocotb.test()(run_test_interleave)


# cocotb-test

tests_dir = os.path.dirname(__file__)
rtl_dir = os.path.abspath(os.path.join(tests_dir, '..', '..', 'rtl'))


def au55c_parameters():
    # parameters as instantiated for the AU55C build (raw_if_pcie_us -> raw_if_pcie -> dma_if_pcie)
    p = {}
    p['TLP_DATA_WIDTH'] = 512
    p['TLP_HDR_WIDTH'] = 128
    p['TLP_SEG_COUNT'] = 1
    p['TX_SEQ_NUM_COUNT'] = 2
    p['TX_SEQ_NUM_WIDTH'] = 5
    p['TX_SEQ_NUM_ENABLE'] = 1
    p['RAM_SEL_WIDTH'] = 2
    p['RAM_ADDR_WIDTH'] = 20
    p['RAM_SEG_COUNT'] = p['TLP_SEG_COUNT']*2
    p['RAM_SEG_DATA_WIDTH'] = p['TLP_DATA_WIDTH']*2 // p['RAM_SEG_COUNT']
    p['RAM_SEG_BE_WIDTH'] = p['RAM_SEG_DATA_WIDTH'] // 8
    p['RAM_SEG_ADDR_WIDTH'] = p['RAM_ADDR_WIDTH'] - (p['RAM_SEG_COUNT']*p['RAM_SEG_BE_WIDTH']-1).bit_length()
    p['PCIE_ADDR_WIDTH'] = 64
    p['PCIE_TAG_COUNT'] = 256
    p['LEN_WIDTH'] = 16
    p['TAG_WIDTH'] = 12
    p['OP_TABLE_SIZE'] = p['PCIE_TAG_COUNT']
    p['TX_LIMIT'] = 2**(6-1)
    p['CPLH_FC_LIMIT'] = 256
    p['CPLD_FC_LIMIT'] = 2048-256
    p['TLP_FORCE_64_BIT_ADDR'] = 1
    p['CHECK_BUS_NUMBER'] = 0
    return p


def run_sim(request, parameters, env):
    dut = "dma_if_pcie_rd"
    module = os.path.splitext(os.path.basename(__file__))[0]
    extra_env = {f'PARAM_{k}': str(v) for k, v in parameters.items()}
    extra_env.update({k: str(v) for k, v in env.items()})
    sim_build = os.path.join(tests_dir, "sim_build",
        request.node.name.replace('[', '-').replace(']', ''))
    cocotb_test.simulator.run(
        python_search=[tests_dir],
        verilog_sources=[os.path.join(rtl_dir, f"{dut}.v")],
        toplevel=dut,
        module=module,
        parameters=parameters,
        sim_build=sim_build,
        extra_env=extra_env,
    )


def test_dma_if_pcie_rd_interleave(request):
    run_sim(request, au55c_parameters(), {"SEED": os.getenv("SEED", "1")})
