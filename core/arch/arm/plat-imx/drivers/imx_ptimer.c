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
#define CNTP_CTL_ENABLE		BIT(0)
#define CNTP_CTL_IMASK		BIT(1)

#define GIC_SPI_SEC_PHY_TIMER	29

static void write_cntp_ctl(uint32_t val){

    vaddr_t ptimer_base = core_mmu_get_va(PTIMER_BASE, MEM_AREA_IO_SEC,
					   PTIMER_SIZE);
	io_write32(ptimer_base+0x8, val);
	return;
}


static void write_cntp_tval(uint32_t val){

    vaddr_t ptimer_base = core_mmu_get_va(PTIMER_BASE, MEM_AREA_IO_SEC,
					   PTIMER_SIZE);
	io_write32(ptimer_base, val);
	return;
}

static uint32_t read_cntp_ctl(void){

	vaddr_t ptimer_base = core_mmu_get_va(PTIMER_BASE, MEM_AREA_IO_SEC,
					   PTIMER_SIZE);
	uint32_t val = (ptimer_base+0x8);
	return val;
}


static uint32_t read_cntfrq(void){
	// hard coding for now
	return 754000000;
}


/* A function to arm programmed timer countdown */
static void arm_timer(void)
{
	if (!timer_val)
		return;

	write_cntp_ctl(read_cntp_ctl() & ~(CNTP_CTL_ENABLE | CNTP_CTL_IMASK));
	write_cntp_tval(timer_val);
	write_cntp_ctl(read_cntp_ctl() | CNTP_CTL_ENABLE);
}

/* A function to load an periodic delay and arm the timer */
static void arm_timer_with_period(unsigned int period_sec)
{
	uint32_t countdown = 0;

	if (MUL_OVERFLOW(period_sec, read_cntfrq(), &countdown))
		panic();

	timer_val = countdown;
	arm_timer();
}
/* A handler for the timer interrupt */
static enum itr_return arm_cntp_it_handler(struct itr_handler *handler __unused)
{
	if (timer_val) {
		/* Arm timer again */
		arm_timer();
		/* Do something */
		IMSG("tick");
	}

	return ITRR_HANDLED;
}
static struct itr_handler arm_cntp_handler = {
	.it = GIC_SPI_SEC_PHY_TIMER,
	.handler = arm_cntp_it_handler,
};
KEEP_PAGER(arm_cntp_handler);

/* An init step to register resources and arm a 1sec periodic interrupt */
static TEE_Result init_arm_cntp_timer(void)
{
	itr_add(&arm_cntp_handler);
	itr_enable(arm_cntp_handler.it);
	arm_timer_with_period(1);

	return TEE_SUCCESS;
}
driver_init(init_arm_cntp_timer);
