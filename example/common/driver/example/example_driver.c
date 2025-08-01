// SPDX-License-Identifier: MIT
/*
 * Copyright (c) 2018-2021 Alex Forencich
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "example_driver.h"
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/version.h>
#include <linux/delay.h>

#include <asm/tsc.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)
#include <linux/pci-aspm.h>
#endif

MODULE_DESCRIPTION("verilog-pcie example driver");
MODULE_AUTHOR("Alex Forencich");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_VERSION(DRIVER_VERSION);

static int edev_probe(struct pci_dev *pdev, const struct pci_device_id *ent);
static void edev_remove(struct pci_dev *pdev);
static void edev_shutdown(struct pci_dev *pdev);

static int enumerate_bars(struct example_dev *edev, struct pci_dev *pdev);
static int map_bars(struct example_dev *edev, struct pci_dev *pdev);
static void free_bars(struct example_dev *edev, struct pci_dev *pdev);

static const struct pci_device_id pci_ids[] = {
	{PCI_DEVICE(0x1234, 0x0001)},
	{0 /* end */ }
};

MODULE_DEVICE_TABLE(pci, pci_ids);

static void dma_block_read(struct example_dev *edev,
		dma_addr_t dma_addr, size_t dma_offset,
		size_t dma_offset_mask, size_t dma_stride,
		size_t ram_addr, size_t ram_offset,
		size_t ram_offset_mask, size_t ram_stride,
		size_t block_len, size_t block_count)
{
	unsigned long t;

	// DMA base address
	iowrite32(dma_addr & 0xffffffff, edev->bar[0] + 0x001080);
	iowrite32((dma_addr >> 32) & 0xffffffff, edev->bar[0] + 0x001084);
	// DMA offset address
	iowrite32(dma_offset & 0xffffffff, edev->bar[0] + 0x001088);
	iowrite32((dma_offset >> 32) & 0xffffffff, edev->bar[0] + 0x00108c);
	// DMA offset mask
	iowrite32(dma_offset_mask & 0xffffffff, edev->bar[0] + 0x001090);
	iowrite32((dma_offset_mask >> 32) & 0xffffffff, edev->bar[0] + 0x001094);
	// DMA stride
	iowrite32(dma_stride & 0xffffffff, edev->bar[0] + 0x001098);
	iowrite32((dma_stride >> 32) & 0xffffffff, edev->bar[0] + 0x00109c);
	// RAM base address
	iowrite32(ram_addr & 0xffffffff, edev->bar[0] + 0x0010c0);
	iowrite32((ram_addr >> 32) & 0xffffffff, edev->bar[0] + 0x0010c4);
	// RAM offset address
	iowrite32(ram_offset & 0xffffffff, edev->bar[0] + 0x0010c8);
	iowrite32((ram_offset >> 32) & 0xffffffff, edev->bar[0] + 0x0010cc);
	// RAM offset mask
	iowrite32(ram_offset_mask & 0xffffffff, edev->bar[0] + 0x0010d0);
	iowrite32((ram_offset_mask >> 32) & 0xffffffff, edev->bar[0] + 0x0010d4);
	// RAM stride
	iowrite32(ram_stride & 0xffffffff, edev->bar[0] + 0x0010d8);
	iowrite32((ram_stride >> 32) & 0xffffffff, edev->bar[0] + 0x0010dc);
	// clear cycle count
	iowrite32(0, edev->bar[0] + 0x001008);
	iowrite32(0, edev->bar[0] + 0x00100c);
	// block length
	iowrite32(block_len, edev->bar[0] + 0x001010);
	// block count
	iowrite32(block_count, edev->bar[0] + 0x001018);
	// start
	iowrite32(1, edev->bar[0] + 0x001000);

	// wait for transfer to complete
	t = jiffies + msecs_to_jiffies(20000);
	while (time_before(jiffies, t)) {
		if ((ioread32(edev->bar[0] + 0x001000) & 1) == 0)
			break;
	}

	if ((ioread32(edev->bar[0] + 0x001000) & 1) != 0)
		dev_warn(edev->dev, "%s: operation timed out", __func__);
	if ((ioread32(edev->bar[0] + 0x000000) & 0x300) != 0)
		dev_warn(edev->dev, "%s: DMA engine busy", __func__);
}

static void dma_block_write(struct example_dev *edev,
		dma_addr_t dma_addr, size_t dma_offset,
		size_t dma_offset_mask, size_t dma_stride,
		size_t ram_addr, size_t ram_offset,
		size_t ram_offset_mask, size_t ram_stride,
		size_t block_len, size_t block_count)
{
	unsigned long t;

	// DMA base address
	iowrite32(dma_addr & 0xffffffff, edev->bar[0] + 0x001180);
	iowrite32((dma_addr >> 32) & 0xffffffff, edev->bar[0] + 0x001184);
	// DMA offset address
	iowrite32(dma_offset & 0xffffffff, edev->bar[0] + 0x001188);
	iowrite32((dma_offset >> 32) & 0xffffffff, edev->bar[0] + 0x00118c);
	// DMA offset mask
	iowrite32(dma_offset_mask & 0xffffffff, edev->bar[0] + 0x001190);
	iowrite32((dma_offset_mask >> 32) & 0xffffffff, edev->bar[0] + 0x001194);
	// DMA stride
	iowrite32(dma_stride & 0xffffffff, edev->bar[0] + 0x001198);
	iowrite32((dma_stride >> 32) & 0xffffffff, edev->bar[0] + 0x00119c);
	// RAM base address
	iowrite32(ram_addr & 0xffffffff, edev->bar[0] + 0x0011c0);
	iowrite32((ram_addr >> 32) & 0xffffffff, edev->bar[0] + 0x0011c4);
	// RAM offset address
	iowrite32(ram_offset & 0xffffffff, edev->bar[0] + 0x0011c8);
	iowrite32((ram_offset >> 32) & 0xffffffff, edev->bar[0] + 0x0011cc);
	// RAM offset mask
	iowrite32(ram_offset_mask & 0xffffffff, edev->bar[0] + 0x0011d0);
	iowrite32((ram_offset_mask >> 32) & 0xffffffff, edev->bar[0] + 0x0011d4);
	// RAM stride
	iowrite32(ram_stride & 0xffffffff, edev->bar[0] + 0x0011d8);
	iowrite32((ram_stride >> 32) & 0xffffffff, edev->bar[0] + 0x0011dc);
	// clear cycle count
	iowrite32(0, edev->bar[0] + 0x001108);
	iowrite32(0, edev->bar[0] + 0x00110c);
	// block length
	iowrite32(block_len, edev->bar[0] + 0x001110);
	// block count
	iowrite32(block_count, edev->bar[0] + 0x001118);
	// start
	iowrite32(1, edev->bar[0] + 0x001100);

	// wait for transfer to complete
	t = jiffies + msecs_to_jiffies(20000);
	while (time_before(jiffies, t)) {
		if ((ioread32(edev->bar[0] + 0x001100) & 1) == 0)
			break;
	}

	if ((ioread32(edev->bar[0] + 0x001100) & 1) != 0)
		dev_warn(edev->dev, "%s: operation timed out", __func__);
	if ((ioread32(edev->bar[0] + 0x000000) & 0x300) != 0)
		dev_warn(edev->dev, "%s: DMA engine busy", __func__);
}

static void dma_block_read_bench(struct example_dev *edev,
		dma_addr_t dma_addr, u64 size, u64 stride, u64 count)
{
	u64 cycles;
	u32 rd_req;
	u32 rd_cpl;

	udelay(5);

	rd_req = ioread32(edev->bar[0] + 0x000020);
	rd_cpl = ioread32(edev->bar[0] + 0x000024);

	dma_block_read(edev, dma_addr, 0, 0x3fff, stride,
			0, 0, 0x3fff, stride, size, count);

	cycles = ioread32(edev->bar[0] + 0x001008);

	udelay(5);

	rd_req = ioread32(edev->bar[0] + 0x000020) - rd_req;
	rd_cpl = ioread32(edev->bar[0] + 0x000024) - rd_cpl;

	dev_info(edev->dev, "read %lld blocks of %lld bytes (total %lld B, stride %lld) in %lld ns (%d req %d cpl): %lld Mbps",
			count, size, count*size, stride, cycles * 4, rd_req, rd_cpl, size * count * 8 * 1000 / (cycles * 4));
}

static void dma_block_write_bench(struct example_dev *edev,
		dma_addr_t dma_addr, u64 size, u64 stride, u64 count)
{
	u64 cycles;
	u32 wr_req;

	udelay(5);

	wr_req = ioread32(edev->bar[0] + 0x000028);

	dma_block_write(edev, dma_addr, 0, 0x3fff, stride,
			0, 0, 0x3fff, stride, size, count);

	cycles = ioread32(edev->bar[0] + 0x001108);

	udelay(5);

	wr_req = ioread32(edev->bar[0] + 0x000028) - wr_req;

	dev_info(edev->dev, "wrote %lld blocks of %lld bytes (total %lld B, stride %lld) in %lld ns (%d req): %lld Mbps",
			count, size, count*size, stride, cycles * 4, wr_req, size * count * 8 * 1000 / (cycles * 4));
}

static void dma_cpl_buf_test(struct example_dev *edev, dma_addr_t dma_addr,
		u64 size, u64 stride, u64 count, int stall)
{
	unsigned long t;
	u64 cycles;
	u32 rd_req;
	u32 rd_cpl;

	rd_req = ioread32(edev->bar[0] + 0x000020);
	rd_cpl = ioread32(edev->bar[0] + 0x000024);

	// DMA base address
	iowrite32(dma_addr & 0xffffffff, edev->bar[0] + 0x001080);
	iowrite32((dma_addr >> 32) & 0xffffffff, edev->bar[0] + 0x001084);
	// DMA offset address
	iowrite32(0, edev->bar[0] + 0x001088);
	iowrite32(0, edev->bar[0] + 0x00108c);
	// DMA offset mask
	iowrite32(0x3fff, edev->bar[0] + 0x001090);
	iowrite32(0, edev->bar[0] + 0x001094);
	// DMA stride
	iowrite32(stride & 0xffffffff, edev->bar[0] + 0x001098);
	iowrite32((stride >> 32) & 0xffffffff, edev->bar[0] + 0x00109c);
	// RAM base address
	iowrite32(0, edev->bar[0] + 0x0010c0);
	iowrite32(0, edev->bar[0] + 0x0010c4);
	// RAM offset address
	iowrite32(0, edev->bar[0] + 0x0010c8);
	iowrite32(0, edev->bar[0] + 0x0010cc);
	// RAM offset mask
	iowrite32(0x3fff, edev->bar[0] + 0x0010d0);
	iowrite32(0, edev->bar[0] + 0x0010d4);
	// RAM stride
	iowrite32(stride & 0xffffffff, edev->bar[0] + 0x0010d8);
	iowrite32((stride >> 32) & 0xffffffff, edev->bar[0] + 0x0010dc);
	// clear cycle count
	iowrite32(0, edev->bar[0] + 0x001008);
	iowrite32(0, edev->bar[0] + 0x00100c);
	// block length
	iowrite32(size, edev->bar[0] + 0x001010);
	// block count
	iowrite32(count, edev->bar[0] + 0x001018);

	if (stall)
		iowrite32(stall, edev->bar[0] + 0x000040);

	// start
	iowrite32(1, edev->bar[0] + 0x001000);

	if (stall)
		msleep(10);

	// wait for transfer to complete
	t = jiffies + msecs_to_jiffies(20000);
	while (time_before(jiffies, t)) {
		if ((ioread32(edev->bar[0] + 0x001000) & 1) == 0)
			break;
	}

	if ((ioread32(edev->bar[0] + 0x001000) & 1) != 0)
		dev_warn(edev->dev, "%s: operation timed out", __func__);
	if ((ioread32(edev->bar[0] + 0x000000) & 0x300) != 0)
		dev_warn(edev->dev, "%s: DMA engine busy", __func__);

	cycles = ioread32(edev->bar[0] + 0x001008);

	rd_req = ioread32(edev->bar[0] + 0x000020) - rd_req;
	rd_cpl = ioread32(edev->bar[0] + 0x000024) - rd_cpl;

	dev_info(edev->dev, "read %lld x %lld B (total %lld B %lld CPLD, stride %lld) in %lld ns (%d req %d cpl): %lld Mbps",
			count, size, count*size, count*((size+15) / 16), stride, cycles * 4, rd_req, rd_cpl, size * count * 8 * 1000 / (cycles * 4));
}

static irqreturn_t edev_intr(int irq, void *data)
{
	struct example_dev *edev = data;
	struct device *dev = &edev->pdev->dev;

	edev->irqcount++;

	dev_info(dev, "Interrupt");

	return IRQ_HANDLED;
}

static int edev_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	int ret = 0;
	struct example_dev *edev;
	struct device *dev = &pdev->dev;

	int k;
	int mismatch = 0;

	dev_info(dev, DRIVER_NAME " probe");
	dev_info(dev, " Vendor: 0x%04x", pdev->vendor);
	dev_info(dev, " Device: 0x%04x", pdev->device);
	dev_info(dev, " Subsystem vendor: 0x%04x", pdev->subsystem_vendor);
	dev_info(dev, " Subsystem device: 0x%04x", pdev->subsystem_device);
	dev_info(dev, " Class: 0x%06x", pdev->class);
	dev_info(dev, " PCI ID: %04x:%02x:%02x.%d", pci_domain_nr(pdev->bus),
			pdev->bus->number, PCI_SLOT(pdev->devfn), PCI_FUNC(pdev->devfn));
	if (pdev->pcie_cap) {
		u16 devctl;
		u32 lnkcap;
		u16 lnkctl;
		u16 lnksta;

		pci_read_config_word(pdev, pdev->pcie_cap + PCI_EXP_DEVCTL, &devctl);
		pci_read_config_dword(pdev, pdev->pcie_cap + PCI_EXP_LNKCAP, &lnkcap);
		pci_read_config_word(pdev, pdev->pcie_cap + PCI_EXP_LNKCTL, &lnkctl);
		pci_read_config_word(pdev, pdev->pcie_cap + PCI_EXP_LNKSTA, &lnksta);

		dev_info(dev, " Max payload size: %d bytes",
				128 << ((devctl & PCI_EXP_DEVCTL_PAYLOAD) >> 5));
		dev_info(dev, " Max read request size: %d bytes",
				128 << ((devctl & PCI_EXP_DEVCTL_READRQ) >> 12));
		dev_info(dev, " Read completion boundary: %d bytes",
				lnkctl & PCI_EXP_LNKCTL_RCB ? 128 : 64);
		dev_info(dev, " Link capability: gen %d x%d",
				lnkcap & PCI_EXP_LNKCAP_SLS, (lnkcap & PCI_EXP_LNKCAP_MLW) >> 4);
		dev_info(dev, " Link status: gen %d x%d",
				lnksta & PCI_EXP_LNKSTA_CLS, (lnksta & PCI_EXP_LNKSTA_NLW) >> 4);
		dev_info(dev, " Relaxed ordering: %s",
				devctl & PCI_EXP_DEVCTL_RELAX_EN ? "enabled" : "disabled");
		dev_info(dev, " Phantom functions: %s",
				devctl & PCI_EXP_DEVCTL_PHANTOM ? "enabled" : "disabled");
		dev_info(dev, " Extended tags: %s",
				devctl & PCI_EXP_DEVCTL_EXT_TAG ? "enabled" : "disabled");
		dev_info(dev, " No snoop: %s",
				devctl & PCI_EXP_DEVCTL_NOSNOOP_EN ? "enabled" : "disabled");
	}
#ifdef CONFIG_NUMA
	dev_info(dev, " NUMA node: %d", pdev->dev.numa_node);
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 17, 0)
	pcie_print_link_status(pdev);
#endif

	edev = devm_kzalloc(dev, sizeof(struct example_dev), GFP_KERNEL);
	if (!edev)
		return -ENOMEM;

	edev->pdev = pdev;
	edev->dev = dev;
	pci_set_drvdata(pdev, edev);

	// Allocate DMA buffer
	edev->dma_region_len = 16 * 1024;
	edev->dma_region = dma_alloc_coherent(dev, edev->dma_region_len,
			&edev->dma_region_addr, GFP_KERNEL | __GFP_ZERO);
	if (!edev->dma_region) {
		ret = -ENOMEM;
		goto fail_dma_alloc;
	}

	dev_info(dev, "Allocated DMA region virt %p, phys %p",
			edev->dma_region, (void *)edev->dma_region_addr);

	// Disable ASPM
	pci_disable_link_state(pdev, PCIE_LINK_STATE_L0S |
			PCIE_LINK_STATE_L1 | PCIE_LINK_STATE_CLKPM);

	// Enable device
	ret = pci_enable_device_mem(pdev);
	if (ret) {
		dev_err(dev, "Failed to enable PCI device");
		goto fail_enable_device;
	}

	// Enable bus mastering for DMA
	pci_set_master(pdev);

	// Reserve regions
	ret = pci_request_regions(pdev, DRIVER_NAME);
	if (ret) {
		dev_err(dev, "Failed to reserve regions");
		goto fail_regions;
	}

	// Enumerate BARs
	enumerate_bars(edev, pdev);

	// Map BARs
	ret = map_bars(edev, pdev);
	if (ret) {
		dev_err(dev, "Failed to map BARs");
		goto fail_map_bars;
	}

	// Allocate MSI IRQs
	ret = pci_alloc_irq_vectors(pdev, 1, 32, PCI_IRQ_MSI | PCI_IRQ_MSIX);
	if (ret < 0) {
		dev_err(dev, "Failed to allocate IRQs");
		goto fail_map_bars;
	}

	// Set up interrupt
	ret = pci_request_irq(pdev, 0, edev_intr, 0, edev, DRIVER_NAME);
	if (ret < 0) {
		dev_err(dev, "Failed to request IRQ");
		goto fail_irq;
	}

	// Read/write test
	dev_info(dev, "write to BAR2");
	iowrite32(0x11223344, edev->bar[2]);

	dev_info(dev, "read from BAR2");
	// fence
	mb();
	// Get tsc clock:
	int ts = rdtsc();
	int val = ioread32(edev->bar[2]);
	ts = rdtsc() - ts;
	mb();
	dev_info(dev, "%08x", val);
	dev_info(dev, "TSC clock delta: %d", ts);
	
	// Test latency for many reads
	int nb_reads;
	for (nb_reads = 1; nb_reads < 100000; nb_reads *= 10) {
		dev_info(dev, "test latency for %d reads", nb_reads);

		mb();
		ts = rdtsc();
		for (k = 0; k < nb_reads; k++) {
			ioread32(edev->bar[2] + k * 4);
		}
		ts = rdtsc() - ts;
		mb();

		dev_info(dev, "Mean latency for %d reads: %d", nb_reads, ts / nb_reads);
	}

	// PCIe DMA test
	dev_info(dev, "write test data");
	for (k = 0; k < 256; k++)
		((char *)edev->dma_region)[k] = k;

	dev_info(dev, "read test data");
	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 1,
			edev->dma_region, 256, true);

	dev_info(dev, "check DMA enable");
	dev_info(dev, "%08x", ioread32(edev->bar[0] + 0x000000));

	dev_info(dev, "enable DMA");
	iowrite32(0x1, edev->bar[0] + 0x000000);

	dev_info(dev, "check DMA enable");
	dev_info(dev, "%08x", ioread32(edev->bar[0] + 0x000000));

	dev_info(dev, "enable interrupts");
	iowrite32(0x3, edev->bar[0] + 0x000008);

	dev_info(dev, "start copy to card");
	iowrite32((edev->dma_region_addr + 0x0000) & 0xffffffff, edev->bar[0] + 0x000100);
	iowrite32(((edev->dma_region_addr + 0x0000) >> 32) & 0xffffffff, edev->bar[0] + 0x000104);
	iowrite32(0x100, edev->bar[0] + 0x000108);
	iowrite32(0, edev->bar[0] + 0x00010C);  // This seems unnecessary, it will be ignored by the hardware.
	iowrite32(0x100, edev->bar[0] + 0x000110);
	iowrite32(0xAA, edev->bar[0] + 0x000114);

	msleep(1);

	dev_info(dev, "Read status");
	dev_info(dev, "%08x", ioread32(edev->bar[0] + 0x000000));
	dev_info(dev, "%08x", ioread32(edev->bar[0] + 0x000118));

	dev_info(dev, "start copy to host");
	iowrite32((edev->dma_region_addr + 0x0200) & 0xffffffff, edev->bar[0] + 0x000200);
	iowrite32(((edev->dma_region_addr + 0x0200) >> 32) & 0xffffffff, edev->bar[0] + 0x000204);
	iowrite32(0x100, edev->bar[0] + 0x000208);
	iowrite32(0, edev->bar[0] + 0x00020C);
	iowrite32(0x100, edev->bar[0] + 0x000210);
	iowrite32(0x55, edev->bar[0] + 0x000214);

	msleep(1);

	dev_info(dev, "Read status");
	dev_info(dev, "%08x", ioread32(edev->bar[0] + 0x000000));
	dev_info(dev, "%08x", ioread32(edev->bar[0] + 0x000218));

	dev_info(dev, "read test data");
	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 1,
			edev->dma_region + 0x0200, 256, true);

	if (memcmp(edev->dma_region + 0x0000, edev->dma_region + 0x0200, 256) == 0) {
		dev_info(dev, "test data matches");
	} else {
		dev_warn(dev, "test data mismatch");
		mismatch = 1;
	}

	dev_info(dev, "start immediate write to host");
	iowrite32((edev->dma_region_addr + 0x0200) & 0xffffffff, edev->bar[0] + 0x000200);
	iowrite32(((edev->dma_region_addr + 0x0200) >> 32) & 0xffffffff, edev->bar[0] + 0x000204);
	iowrite32(0x44332211, edev->bar[0] + 0x000208);
	iowrite32(0, edev->bar[0] + 0x00020C);
	iowrite32(0x4, edev->bar[0] + 0x000210);
	iowrite32(0x800000AA, edev->bar[0] + 0x000214);

	msleep(1);

	dev_info(dev, "Read status");
	dev_info(dev, "%08x", ioread32(edev->bar[0] + 0x000000));
	dev_info(dev, "%08x", ioread32(edev->bar[0] + 0x000218));

	dev_info(dev, "read data");
	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 1,
			edev->dma_region + 0x0200, 4, true);

	if (!mismatch) {
		u64 size;
		u64 stride;
		u64 count;

		dev_info(dev, "disable interrupts");
		iowrite32(0x0, edev->bar[0] + 0x000008);

		dev_info(dev, "test RX completion buffer (CPLH, 8)");

		size = 8;
		stride = size;
		for (count = 32; count <= 256; count += 8) {
			dma_cpl_buf_test(edev,
					edev->dma_region_addr + 0x0000,
					size, stride, count, 100000);
			if ((ioread32(edev->bar[0] + 0x000000) & 0x300) != 0)
				goto out;
		}

		dev_info(dev, "test RX completion buffer (CPLH, unaligned 8+64)");

		size = 8+64;
		stride = 0;
		for (count = 8; count <= 256; count += 8) {
			dma_cpl_buf_test(edev,
					edev->dma_region_addr + 128 - 8,
					size, stride, count, 400000);
			if ((ioread32(edev->bar[0] + 0x000000) & 0x300) != 0)
				goto out;
		}

		dev_info(dev, "test RX completion buffer (CPLH, unaligned 8+128+8)");

		size = 8+128+8;
		stride = 0;
		for (count = 8; count <= 256; count += 8) {
			dma_cpl_buf_test(edev,
					edev->dma_region_addr + 128 - 8,
					size, stride, count, 100000);
			if ((ioread32(edev->bar[0] + 0x000000) & 0x300) != 0)
				goto out;
		}

		dev_info(dev, "test RX completion buffer (CPLD)");

		size = 512;
		stride = size;
		for (count = 8; count <= 256; count += 8) {
			dma_cpl_buf_test(edev,
					edev->dma_region_addr + 0x0000,
					size, stride, count, 100000);
			if ((ioread32(edev->bar[0] + 0x000000) & 0x300) != 0)
				goto out;
		}

		dev_info(dev, "perform block reads (dma_alloc_coherent)");

		count = 10000;
		for (size = 1; size <= 8192; size *= 2) {
			for (stride = size; stride <= max(size, 256llu); stride *= 2) {
				dma_block_read_bench(edev,
						edev->dma_region_addr + 0x0000,
						size, stride, count);
				if ((ioread32(edev->bar[0] + 0x000000) & 0x300) != 0)
					goto out;
			}
		}

		dev_info(dev, "perform block writes (dma_alloc_coherent)");

		count = 10000;
		for (size = 1; size <= 8192; size *= 2) {
			for (stride = size; stride <= max(size, 256llu); stride *= 2) {
				dma_block_write_bench(edev,
						edev->dma_region_addr + 0x0000,
						size, stride, count);
				if ((ioread32(edev->bar[0] + 0x000000) & 0x300) != 0)
					goto out;
			}
		}
	}

out:
	dev_info(dev, "Read status");
	dev_info(dev, "%08x", ioread32(edev->bar[0] + 0x000000));

	// probe complete
	return 0;

	// error handling
fail_irq:
	pci_free_irq_vectors(pdev);
fail_map_bars:
	free_bars(edev, pdev);
	pci_release_regions(pdev);
fail_regions:
	pci_clear_master(pdev);
	pci_disable_device(pdev);
fail_enable_device:
	dma_free_coherent(dev, edev->dma_region_len, edev->dma_region, edev->dma_region_addr);
fail_dma_alloc:
	return ret;
}

static void edev_remove(struct pci_dev *pdev)
{
	struct example_dev *edev = pci_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	dev_info(dev, DRIVER_NAME " remove");

	pci_free_irq(pdev, 0, edev);
	pci_free_irq_vectors(pdev);
	free_bars(edev, pdev);
	pci_release_regions(pdev);
	pci_clear_master(pdev);
	pci_disable_device(pdev);
	dma_free_coherent(dev, edev->dma_region_len, edev->dma_region, edev->dma_region_addr);
}

static void edev_shutdown(struct pci_dev *pdev)
{
	dev_info(&pdev->dev, DRIVER_NAME " shutdown");

	edev_remove(pdev);
}

static int enumerate_bars(struct example_dev *edev, struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	int i;

	for (i = 0; i < 6; i++) {
		resource_size_t bar_start = pci_resource_start(pdev, i);

		if (bar_start) {
			resource_size_t bar_end = pci_resource_end(pdev, i);
			unsigned long bar_flags = pci_resource_flags(pdev, i);

			dev_info(dev, "BAR[%d] 0x%08llx-0x%08llx flags 0x%08lx",
					i, bar_start, bar_end, bar_flags);
		}
	}

	return 0;
}

static int map_bars(struct example_dev *edev, struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	int i;

	for (i = 0; i < 6; i++) {
		resource_size_t bar_start = pci_resource_start(pdev, i);
		resource_size_t bar_end = pci_resource_end(pdev, i);
		resource_size_t bar_len = bar_end - bar_start + 1;

		edev->bar_len[i] = bar_len;

		if (!bar_start || !bar_end) {
			edev->bar_len[i] = 0;
			continue;
		}

		if (bar_len < 1) {
			dev_warn(dev, "BAR[%d] is less than 1 byte", i);
			continue;
		}

		edev->bar[i] = pci_ioremap_bar(pdev, i);

		if (!edev->bar[i]) {
			dev_err(dev, "Could not map BAR[%d]", i);
			return -1;
		}

		dev_info(dev, "BAR[%d] mapped at 0x%p with length %llu",
			i, edev->bar[i], bar_len);
	}

	return 0;
}

static void free_bars(struct example_dev *edev, struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	int i;

	for (i = 0; i < 6; i++) {
		if (edev->bar[i]) {
			pci_iounmap(pdev, edev->bar[i]);
			edev->bar[i] = NULL;
			dev_info(dev, "Unmapped BAR[%d]", i);
		}
	}
}

static struct pci_driver pci_driver = {
	.name = DRIVER_NAME,
	.id_table = pci_ids,
	.probe = edev_probe,
	.remove = edev_remove,
	.shutdown = edev_shutdown
};

static int __init edev_init(void)
{
	printk(KERN_INFO DRIVER_NAME " driver version %s\n", DRIVER_VERSION);
	return pci_register_driver(&pci_driver);
}

static void __exit edev_exit(void)
{
	pci_unregister_driver(&pci_driver);
}

module_init(edev_init);
module_exit(edev_exit);
