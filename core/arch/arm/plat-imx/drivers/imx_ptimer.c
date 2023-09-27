/* Source: https://github.com/OP-TEE/optee_os/issues/2925  */
/* ARM Documentation */
/* https://developer.arm.com/documentation/ddi0407/e/global-timer--private-timers--and-watchdog-registers/global-timer-registers/global-timer-control-register?lang=en */

#include <io.h>
#include <imx.h>
#include <arm.h>
#include <imx-regs.h>
#include <initcall.h>
#include <kernel/interrupt.h>
#include <mm/core_memprot.h>

/* Timer countdown/delay argument for the target calibration periodicity */
static uint64_t timer_val, compare_value;
static uint32_t timer_tval_low;
static uint32_t timer_tval_high;
static uint32_t timer_cval_low;
static uint32_t timer_cval_high;

#define PTIMER_BASE 0x00A00200
#define PTIMER_SIZE 0xFF
#define PTIMER_ENABLE				BIT(0)
#define PTIMER_CTL_ENABLE			BIT(1)
#define PTIMER_CTL_INT_ENABLE		BIT(2)
#define PTIMER_CTL_SINGLE_SHOT		BIT(3)
#define PTIMER_BOOT_PRE_SCALER		0xFF00

#define GIC_SPI_SEC_PHY_TIMER	27

uint32_t get_hi(uint64_t x){
	return x >> 32;
}

uint32_t get_lo(uint64_t x){
	return ((1ULL << 32) - 1) & x;
}

static void clear_timer_interrupt(void){

	vaddr_t ptimer_base = core_mmu_get_va(PTIMER_BASE, MEM_AREA_IO_SEC,
					   PTIMER_SIZE);
	io_write32(ptimer_base+U(0x0C), BIT(0));
	return;

}

static void write_ptimer_ctl(uint32_t val){

    vaddr_t ptimer_base = core_mmu_get_va(PTIMER_BASE, MEM_AREA_IO_SEC,
					   PTIMER_SIZE);
	io_write32(ptimer_base+U(0x08), val);
	return;
}

static void write_ptimer_cval_high(uint32_t val){

    vaddr_t ptimer_base = core_mmu_get_va(PTIMER_BASE, MEM_AREA_IO_SEC,
					   PTIMER_SIZE);
	io_write32(ptimer_base+U(0x14), val);
	return;
}

static void write_ptimer_cval_low(uint32_t val){

    vaddr_t ptimer_base = core_mmu_get_va(PTIMER_BASE, MEM_AREA_IO_SEC,
					   PTIMER_SIZE);
	io_write32(ptimer_base+U(0x10), val);
	return;
}

static uint32_t read_ptimer_ctl(void){

	vaddr_t ptimer_base = core_mmu_get_va(PTIMER_BASE, MEM_AREA_IO_SEC,
					   PTIMER_SIZE);
	uint32_t val = io_read32(ptimer_base+U(0x08));
	return val;
}

static uint32_t read_ptimer_tval_low(void){

	vaddr_t ptimer_base = core_mmu_get_va(PTIMER_BASE, MEM_AREA_IO_SEC,
					   PTIMER_SIZE);
	uint32_t val = io_read32(ptimer_base);
	return val;
}

static uint32_t read_ptimer_tval_high(void){

	vaddr_t ptimer_base = core_mmu_get_va(PTIMER_BASE, MEM_AREA_IO_SEC,
					   PTIMER_SIZE);
	uint32_t val = io_read32(ptimer_base+U(0x04));
	return val;
}

/* A function to arm programmed timer countdown */
static void arm_timer(void)
{
	if (!timer_val)
		return;

	// read ptimer_value
	timer_tval_high = read_ptimer_tval_high();
	timer_tval_low = read_ptimer_tval_low();
	//if(!(timer_high == read_ptimer_tval_high())){
	//	IMSG("Incorrect timer values -- high: %x, low: %x", timer_high, timer_low);
	//	timer_val = 0;
	//	return;
	//}
	IMSG("Current timer values -- high: %x, low: %x", timer_tval_high, timer_tval_low);

	// computing the compare value
	compare_value = (uint64_t) timer_tval_high;
	compare_value = compare_value << 32;
	compare_value = compare_value + (uint64_t) (timer_tval_low);
	compare_value = compare_value + timer_val;

	// writing compare value
	IMSG("Arming with the value -- high: %x, low: %x", get_hi(compare_value), get_lo(compare_value));
	write_ptimer_cval_low(get_lo(compare_value));
	write_ptimer_cval_high(get_hi(compare_value));
	// enabling compare value and the corresponding interrupt 
	write_ptimer_ctl(PTIMER_ENABLE | PTIMER_CTL_INT_ENABLE | PTIMER_CTL_ENABLE);
}

/* A function to load an periodic delay and arm the timer */
static void arm_timer_with_period(unsigned int period_msec)
{

	// hardcoding frequency value now == 996000000
	timer_val = (uint64_t) period_msec * (uint64_t) 996000;

	IMSG("Timer value before arming it: %lx -- high: %x, low: %x", timer_val, get_hi(timer_val), get_lo(timer_val));
	arm_timer();
}


/* A handler for the timer interrupt */
static enum itr_return arm_ptimer_it_handler(struct itr_handler *handler __unused)
{
	// disable the timer and clear the status flag
	clear_timer_interrupt();
	write_ptimer_ctl(PTIMER_ENABLE | PTIMER_CTL_INT_ENABLE);

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
	// sending interrupt to all cpus
	itr_set_affinity(GIC_SPI_SEC_PHY_TIMER, 0x0F);
	// enable global timer if it is not enabled already
	uint32_t timer_status = read_ptimer_ctl();
	if (!(timer_status & 1)){
		IMSG("Global Timer was disabled. Enabling it now !!!");
		write_ptimer_ctl(PTIMER_ENABLE);
	}
	IMSG("Global Timer Enabled.");
	// clear interrupt status flag
	clear_timer_interrupt();
	write_ptimer_ctl(PTIMER_ENABLE | PTIMER_CTL_INT_ENABLE);
	IMSG("Global Timer Interrupt Registered !!!");

	// set timer to fire after given time in milli-seconds
	arm_timer_with_period(5000);

	return TEE_SUCCESS;
}

driver_init(init_arm_ptimer_timer);
