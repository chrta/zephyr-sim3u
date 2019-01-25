/*
 * Copyright (c) 2019, Christian Taedcke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief SoC initialization for the SIM3U
 */

#include <kernel.h>
#include <init.h>
#include <soc.h>

#include <arch/cpu.h>
#include <cortex_m/exc.h>

uint32_t SystemCoreClock = 20000000;

uint32_t SystemCoreClockGet(void)
{
    return SystemCoreClock;
}

/**
 * @brief Initialize the system clock
 *
 * @return N/A
 *
 */
static ALWAYS_INLINE void clkInit(void)
{

}

/**
 * @brief Perform basic hardware initialization
 *
 * Initialize the interrupt controller device drivers.
 * Also initialize the timer device driver, if required.
 *
 * @return 0
 */
static int silabs_sim3u_init(struct device *arg)
{
	ARG_UNUSED(arg);

	unsigned int oldLevel; /* old interrupt lock level */

	/* disable interrupts */
	oldLevel = irq_lock();

	/* disable watchdog reset source */
	RSTSRC_0->RESETEN.bit.WDTREN = 0;

	_ClearFaults();

	/* Initialize system clock */
	clkInit();

	/*
	 * install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise
	 */
	NMI_INIT();

	/* restore interrupt state */
	irq_unlock(oldLevel);
	return 0;
}

SYS_INIT(silabs_sim3u_init, PRE_KERNEL_1, 0);
