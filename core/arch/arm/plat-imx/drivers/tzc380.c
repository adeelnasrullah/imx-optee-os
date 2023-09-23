// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright 2019 Pengutronix
 * All rights reserved.
 * Copyright 2022 NXP
 *
 * Rouven Czerwinski <entwicklung@pengutronix.de>
 */

#include <config.h>
#include <drivers/tzc380.h>
#include <imx-regs.h>
#include <imx.h>
#include <initcall.h>
#include <kernel/pm.h>
#include <kernel/panic.h>
#include <mm/core_memprot.h>
#include <mm/generic_ram_layout.h>
// later additions
#include <assert.h>
#include <trace.h>
#include <kernel/interrupt.h>

/*
 * TZASC2_BASE is asserted non null when used.
 * This is needed to compile the code for i.MX6UL/L
 * and i.MX8MQ.
 */
#ifndef TZASC2_BASE
#define TZASC2_BASE			0
#else
register_phys_mem(MEM_AREA_IO_SEC, TZASC2_BASE, TZASC_SIZE);
#endif

register_phys_mem(MEM_AREA_IO_SEC, TZASC_BASE, TZASC_SIZE);

#define PTIMER_BASE 0x00A00600
#define PTIMER_SIZE 0xFF
#define PTIMER_CTL_ENABLE			BIT(0)
#define PTIMER_CTL_SINGLE_SHOT		BIT(1)
#define PTIMER_CTL_INT_ENABLE		BIT(2)
#define PTIMER_BOOT_PRE_SCALER		0xFF00

void write_ptimer_ctl(uint32_t val){

    vaddr_t ptimer_base = core_mmu_get_va(PTIMER_BASE, MEM_AREA_IO_SEC,
					   PTIMER_SIZE);
	io_write32(ptimer_base+U(0x08), val);
	return;
}

// later additions
static enum itr_return tzc_it_handler(struct itr_handler *handler __unused)
{
	uint32_t r=0;
        asm volatile("mrc p15, 0, %0, c9, c13, 0" : "=r"(r) );
	DMSG("Request received. CPU cycle count: %u", r);
	DMSG("TZC permission failure");
	//EMSG("TZC permission failure");
	//tzc_fail_dump();

	tzc_int_clear();
	write_ptimer_ctl(PTIMER_BOOT_PRE_SCALER | PTIMER_CTL_INT_ENABLE);
	DMSG("Interrupt Cleared. Likely returned from the handler.");

	return ITRR_HANDLED;
}

static struct itr_handler tzc_itr_handler = {
	.it = 140,
	.handler = tzc_it_handler,
};

static struct itr_handler tzc_itr_handler2 = {
	.it = 141,
	.handler = tzc_it_handler,
};

DECLARE_KEEP_PAGER(tzc_itr_handler);
DECLARE_KEEP_PAGER(tzc_itr_handler2);

static int imx_tzc_auto_configure(vaddr_t addr, vaddr_t rsize, uint32_t attr,
				  uint8_t region)
{
	vaddr_t addr_imx = 0;

	/*
	 * On 8mscale platforms, the TZASC controller for the DRAM protection,
	 * has the memory regions starting at address 0x0 instead of the DRAM
	 * base address (0x40000000)
	 */
	if (IS_ENABLED(CFG_MX8M))
		addr_imx = addr - CFG_DRAM_BASE;
	else
		addr_imx = addr;

	return tzc_auto_configure(addr_imx, rsize, attr, region);
}

static TEE_Result imx_configure_tzasc(void)
{
	vaddr_t addr[2] = {0};
	int end = 1;
	int i = 0;

	addr[0] = core_mmu_get_va(TZASC_BASE, MEM_AREA_IO_SEC, 1);

	if (IS_ENABLED(CFG_MX6Q) || IS_ENABLED(CFG_MX6D) ||
	    IS_ENABLED(CFG_MX6DL)) {
		assert(TZASC2_BASE != 0);
		addr[1] = core_mmu_get_va(TZASC2_BASE, MEM_AREA_IO_SEC, 1);
		end = 2;
	}

	DMSG("Physical1: %x, Virtual1: %x", TZASC_BASE, addr[0] );
	DMSG("Physical2: %x, Virtual2: %x", TZASC2_BASE, addr[1] );

	for (i = 0; i < end; i++) {
		uint8_t region = 1;

		tzc_init(addr[i]);

		region = imx_tzc_auto_configure(CFG_DRAM_BASE, CFG_DDR_SIZE,
						TZC_ATTR_SP_NS_RW, region);
		region = imx_tzc_auto_configure(CFG_TZDRAM_START,
						CFG_TZDRAM_SIZE,
						TZC_ATTR_SP_S_RW, region);
		region = imx_tzc_auto_configure(CFG_SHMEM_START, CFG_SHMEM_SIZE,
						TZC_ATTR_SP_ALL, region);

		if (tzc_regions_lockdown() != TEE_SUCCESS)
			panic("Region lockdown failed!");

		tzc_dump_state();
		
		if(i==0){
		itr_add(&tzc_itr_handler);
		itr_enable(tzc_itr_handler.it);
		// fixing it to CPU0
		itr_set_affinity(tzc_itr_handler.it, 0);
		tzc_set_action(TZC_ACTION_INT);
		}else{
		itr_add(&tzc_itr_handler2);
		itr_enable(tzc_itr_handler2.it);
		// fixing it to CPU0
		itr_set_affinity(tzc_itr_handler2.it, 0);
		tzc_set_action(TZC_ACTION_INT);
		}

		DMSG("Action register: %"PRIx32, tzc_get_action());
	}
	
	// testing if the function works here
	tzc_int_clear();

	return TEE_SUCCESS;
}


static TEE_Result pm_enter_resume(enum pm_op op, uint32_t pm_hint __unused,
		const struct pm_callback_handle *pm_handle __unused)
{
	if (op == PM_OP_RESUME)
		imx_configure_tzasc();

	return TEE_SUCCESS;
}

static TEE_Result tzasc_init(void)
{
	imx_configure_tzasc();
	register_pm_driver_cb(pm_enter_resume, NULL, "imx-tzasc");

	return TEE_SUCCESS;
}

driver_init(tzasc_init);
