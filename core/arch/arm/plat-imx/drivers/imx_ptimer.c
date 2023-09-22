/* Source: https://github.com/OP-TEE/optee_os/issues/2925  */

#include <io.h>
#include <imx.h>
#include <arm.h>
#include <imx-regs.h>
#include <initcall.h>
#include <kernel/interrupt.h>
#include <mm/core_memprot.h>

/* Timer countdown/delay argument for the target calibration periodicity */
static uint32_t timer_val;

#define PTIMER_BASE 0x00A00600
#define PTIMER_SIZE 0xFF
#define PTIMER_CTL_ENABLE			BIT(0)
#define PTIMER_CTL_SINGLE_SHOT		BIT(1)
#define PTIMER_CTL_INT_ENABLE		BIT(2)
#define PTIMER_BOOT_PRE_SCALER		BIT(14)

#define GIC_SPI_SEC_PHY_TIMER	29

static void clear_timer_interrupt(void){

	vaddr_t ptimer_base = core_mmu_get_va(PTIMER_BASE, MEM_AREA_IO_SEC,
					   PTIMER_SIZE);
	io_write32(ptimer_base+0x0C, 0);
	return;

}

static void write_ptimer_ctl(uint32_t val){

    vaddr_t ptimer_base = core_mmu_get_va(PTIMER_BASE, MEM_AREA_IO_SEC,
					   PTIMER_SIZE);
	io_write32(ptimer_base+0x8, val);
	return;
}


static void write_ptimer_tval(uint32_t val){

    vaddr_t ptimer_base = core_mmu_get_va(PTIMER_BASE, MEM_AREA_IO_SEC,
					   PTIMER_SIZE);
	io_write32(ptimer_base, val);
	return;
}

static uint32_t read_ptimer_ctl(void){

	vaddr_t ptimer_base = core_mmu_get_va(PTIMER_BASE, MEM_AREA_IO_SEC,
					   PTIMER_SIZE);
	uint32_t val = (ptimer_base+0x8);
	return val;
}

/* A function to arm programmed timer countdown */
static void arm_timer(void)
{
	if (!timer_val)
		return;

	write_ptimer_tval(timer_val);
	write_ptimer_ctl(read_ptimer_ctl() | PTIMER_CTL_ENABLE);
	IMSG("Armed timer with value: %x, with control register reading: %x", timer_val, read_ptimer_ctl());
}

/* A function to load an periodic delay and arm the timer */
static void arm_timer_with_period(unsigned int period_sec)
{

	// at 996 MHz the maximum period will be 4 just over 4 seconds
	if (period_sec > 4){
		period_sec = 4;
	}

	// hardcoding frequency value now
	uint32_t countdown = period_sec*792000000;

	timer_val = countdown;
	arm_timer();
}


/* A handler for the timer interrupt */
static enum itr_return arm_ptimer_it_handler(struct itr_handler *handler __unused)
{
	// disable the timer and clear the status flag
	clear_timer_interrupt();
	write_ptimer_ctl(PTIMER_CTL_INT_ENABLE);

	if (timer_val) {
		/* Arm timer again */
		arm_timer();
		/* Do something */
		IMSG("Secure Tick!!!!!");
	}

	return ITRR_HANDLED;
}
static struct itr_handler arm_ptimer_handler = {
	.it = GIC_SPI_SEC_PHY_TIMER,
	.handler = arm_ptimer_it_handler,
};
KEEP_PAGER(arm_ptimer_handler);

/* An init step to register resources and arm a 1sec periodic interrupt */
static TEE_Result init_arm_ptimer_timer(void)
{
	// enabling the distributor
	itr_add(&arm_ptimer_handler);
	itr_enable(arm_ptimer_handler.it);
	// enable interrupt generation at the private timer registers
	write_ptimer_ctl(PTIMER_BOOT_PRE_SCALER | PTIMER_CTL_INT_ENABLE);
	clear_timer_interrupt();

	// set timer to fire after given time.
	arm_timer_with_period(4);

	return TEE_SUCCESS;
}

driver_init(init_arm_ptimer_timer);
