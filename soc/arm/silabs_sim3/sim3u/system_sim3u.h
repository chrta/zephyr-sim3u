/*
 * Copyright (c) 2019 Christian Taedcke
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SYSTEM_SIM3U_H
#define SYSTEM_SIM3U_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern uint32_t SystemCoreClock;    /**< System Clock Frequency (Core Clock) */

uint32_t SystemCoreClockGet(void);

static __INLINE void SystemCoreClockUpdate(void)
{
  (void)SystemCoreClockGet();
}

void SystemInit(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_SIM3U_H */
