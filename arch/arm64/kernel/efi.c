/*
 * Extensible Firmware Interface
 *
 * Based on Extensible Firmware Interface Specification version 2.4
 *
 * Copyright (C) 2013, 2014 Linaro Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/efi.h>
#include <linux/export.h>
#include <linux/memblock.h>
#include <linux/bootmem.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <asm/cacheflush.h>
#include <asm/efi.h>
#include <asm/tlbflush.h>
#include <asm/mmu_context.h>

struct efi_memory_map memmap;

static efi_runtime_services_t *runtime;

static u64 efi_system_table;

static int uefi_debug __initdata;
static int __init uefi_debug_setup(char *str)
{
	uefi_debug = 1;

	return 0;
}
early_param("uefi_debug", uefi_debug_setup);

static int __init is_normal_ram(efi_memory_desc_t *md)
{
	if (md->attribute & EFI_MEMORY_WB)
		return 1;
	return 0;
}

/*
 * Translate a EFI virtual address into a physical address: this is necessary,
 * as some data members of the EFI system table are virtually remapped after
 * SetVirtualAddressMap() has been called.
 */
static phys_addr_t efi_to_phys(unsigned long addr)
{
	efi_memory_desc_t *md;

	for_each_efi_memory_desc(&memmap, md) {
		if (!(md->attribute & EFI_MEMORY_RUNTIME))
			continue;
		if (md->virt_addr == 0)
			/* no virtual mapping has been installed by the stub */
			break;
		if (md->virt_addr <= addr &&
		    (addr - md->virt_addr) < (md->num_pages << EFI_PAGE_SHIFT))
			return md->phys_addr + addr - md->virt_addr;
	}
	return addr;
}

static int __init uefi_init(void)
{
	efi_char16_t *c16;
	void *config_tables;
	u64 table_size;
	char vendor[100] = "unknown";
	int i, retval;

	efi.systab = early_memremap(efi_system_table,
				    sizeof(efi_system_table_t));
	if (efi.systab == NULL) {
		pr_warn("Unable to map EFI system table.\n");
		return -ENOMEM;
	}

	set_bit(EFI_BOOT, &efi.flags);
	set_bit(EFI_64BIT, &efi.flags);

	/*
	 * Verify the EFI Table
	 */
	if (efi.systab->hdr.signature != EFI_SYSTEM_TABLE_SIGNATURE) {
		pr_err("System table signature incorrect\n");
		retval = -EINVAL;
		goto out;
	}
	if ((efi.systab->hdr.revision >> 16) < 2)
		pr_warn("Warning: EFI system table version %d.%02d, expected 2.00 or greater\n",
			efi.systab->hdr.revision >> 16,
			efi.systab->hdr.revision & 0xffff);

	/* Show what we know for posterity */
	c16 = early_memremap(efi_to_phys(efi.systab->fw_vendor),
			     sizeof(vendor));
	if (c16) {
		for (i = 0; i < (int) sizeof(vendor) - 1 && *c16; ++i)
			vendor[i] = c16[i];
		vendor[i] = '\0';
		early_memunmap(c16, sizeof(vendor));
	}

	pr_info("EFI v%u.%.02u by %s\n",
		efi.systab->hdr.revision >> 16,
		efi.systab->hdr.revision & 0xffff, vendor);

	retval = efi_config_init(NULL);
	if (retval == 0)
		set_bit(EFI_CONFIG_TABLES, &efi.flags);

out:
	early_memunmap(efi.systab,  sizeof(efi_system_table_t));
	return retval;
}

/*
 * Return true for RAM regions we want to permanently reserve.
 */
static __init int is_reserve_region(efi_memory_desc_t *md)
{
	if (!is_normal_ram(md))
		return 0;

	if (md->attribute & EFI_MEMORY_RUNTIME)
		return 1;

	if (md->type == EFI_ACPI_RECLAIM_MEMORY ||
	    md->type == EFI_RESERVED_TYPE)
		return 1;

	return 0;
}

static __init void reserve_regions(void)
{
	efi_memory_desc_t *md;
	u64 paddr, npages, size;

	if (uefi_debug)
		pr_info("Processing EFI memory map:\n");

	for_each_efi_memory_desc(&memmap, md) {
		paddr = md->phys_addr;
		npages = md->num_pages;

		if (uefi_debug) {
			char buf[64];

			pr_info("  0x%012llx-0x%012llx %s",
				paddr, paddr + (npages << EFI_PAGE_SHIFT) - 1,
				efi_md_typeattr_format(buf, sizeof(buf), md));
		}

		memrange_efi_to_native(&paddr, &npages);
		size = npages << PAGE_SHIFT;

		if (is_normal_ram(md))
			early_init_dt_add_memory_arch(paddr, size);

		if (is_reserve_region(md)) {
			memblock_reserve(paddr, size);
			if (uefi_debug)
				pr_cont("*");
		}

		if (uefi_debug)
			pr_cont("\n");
	}

	set_bit(EFI_MEMMAP, &efi.flags);
}

void __init efi_init(void)
{
	struct efi_fdt_params params;

	/* Grab UEFI information placed in FDT by stub */
	if (!efi_get_fdt_params(&params, uefi_debug))
		return;

	efi_system_table = params.system_table;

	memblock_reserve(params.mmap & PAGE_MASK,
			 PAGE_ALIGN(params.mmap_size + (params.mmap & ~PAGE_MASK)));
	memmap.phys_map = (void *)params.mmap;
	memmap.map = early_memremap(params.mmap, params.mmap_size);
	memmap.map_end = memmap.map + params.mmap_size;
	memmap.desc_size = params.desc_size;
	memmap.desc_version = params.desc_ver;

	if (uefi_init() < 0)
		return;

	reserve_regions();
}

void __init efi_idmap_init(void)
{
	if (!efi_enabled(EFI_BOOT))
		return;

	/* boot time idmap_pg_dir is incomplete, so fill in missing parts */
	efi_setup_idmap();
	early_memunmap(memmap.map, memmap.map_end - memmap.map);
}

static int __init remap_region(efi_memory_desc_t *md, void **new)
{
	u64 paddr, vaddr, npages, size;

	paddr = md->phys_addr;
	npages = md->num_pages;
	memrange_efi_to_native(&paddr, &npages);
	size = npages << PAGE_SHIFT;

	if (is_normal_ram(md))
		vaddr = (__force u64)ioremap_cache(paddr, size);
	else
		vaddr = (__force u64)ioremap(paddr, size);

	if (!vaddr) {
		pr_err("Unable to remap 0x%llx pages @ %p\n",
		       npages, (void *)paddr);
		return 0;
	}

	/* adjust for any rounding when EFI and system pagesize differs */
	md->virt_addr = vaddr + (md->phys_addr - paddr);

	if (uefi_debug)
		pr_info("  EFI remap 0x%012llx => %p\n",
			md->phys_addr, (void *)md->virt_addr);

	memcpy(*new, md, memmap.desc_size);
	*new += memmap.desc_size;

	return 1;
}

/*
 * Switch UEFI from an identity map to a kernel virtual map
 */
static int __init arm64_enter_virtual_mode(void)
{
	efi_memory_desc_t *md;
	phys_addr_t virtmap_phys;
	void *virtmap, *virt_md;
	efi_status_t status;
	u64 mapsize;
	int count = 0;
	unsigned long flags;

	if (!efi_enabled(EFI_BOOT)) {
		pr_info("EFI services will not be available.\n");
		return -1;
	}

	mapsize = memmap.map_end - memmap.map;

	if (efi_runtime_disabled()) {
		pr_info("EFI runtime services will be disabled.\n");
		return -1;
	}

	pr_info("Remapping and enabling EFI services.\n");
	/* replace early memmap mapping with permanent mapping */
	memmap.map = (__force void *)ioremap_cache((phys_addr_t)memmap.phys_map,
						   mapsize);
	memmap.map_end = memmap.map + mapsize;

	efi.memmap = &memmap;

	/* Map the runtime regions */
	virtmap = kmalloc(mapsize, GFP_KERNEL);
	if (!virtmap) {
		pr_err("Failed to allocate EFI virtual memmap\n");
		return -1;
	}
	virtmap_phys = virt_to_phys(virtmap);
	virt_md = virtmap;

	for_each_efi_memory_desc(&memmap, md) {
		if (!(md->attribute & EFI_MEMORY_RUNTIME))
			continue;
		if (!remap_region(md, &virt_md))
			goto err_unmap;
		++count;
	}

	efi.systab = (__force void *)efi_lookup_mapped_addr(efi_system_table);
	if (!efi.systab) {
		/*
		 * If we have no virtual mapping for the System Table at this
		 * point, the memory map doesn't cover the physical offset where
		 * it resides. This means the System Table will be inaccessible
		 * to Runtime Services themselves once the virtual mapping is
		 * installed.
		 */
		pr_err("Failed to remap EFI System Table -- buggy firmware?\n");
		goto err_unmap;
	}
	set_bit(EFI_SYSTEM_TABLES, &efi.flags);

	local_irq_save(flags);
	cpu_switch_mm(idmap_pg_dir, &init_mm);

	/* Call SetVirtualAddressMap with the physical address of the map */
	runtime = efi.systab->runtime;
	efi.set_virtual_address_map = runtime->set_virtual_address_map;

	status = efi.set_virtual_address_map(count * memmap.desc_size,
					     memmap.desc_size,
					     memmap.desc_version,
					     (efi_memory_desc_t *)virtmap_phys);
	cpu_set_reserved_ttbr0();
	flush_tlb_all();
	local_irq_restore(flags);

	kfree(virtmap);

	free_boot_services();

	if (status != EFI_SUCCESS) {
		pr_err("Failed to set EFI virtual address map! [%lx]\n",
			status);
		return -1;
	}

	/* Set up runtime services function pointers */
	runtime = efi.systab->runtime;
	efi_native_runtime_setup();
	set_bit(EFI_RUNTIME_SERVICES, &efi.flags);

	efi.runtime_version = efi.systab->hdr.revision;

	return 0;

err_unmap:
	/* unmap all mappings that succeeded: there are 'count' of those */
	for (virt_md = virtmap; count--; virt_md += memmap.desc_size) {
		md = virt_md;
		iounmap((__force void __iomem *)md->virt_addr);
	}
	kfree(virtmap);
	return -1;
}
early_initcall(arm64_enter_virtual_mode);
