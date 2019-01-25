/* This file is a temporary workaround for mapping of the generated information
 * to the current driver definitions.  This will be removed when the drivers
 * are modified to handle the generated information, or the mapping of
 * generated data matches the driver definitions.
 */

/* SoC level DTS fixup file */

#define DT_NUM_IRQ_PRIO_BITS	DT_ARM_V7M_NVIC_E000E100_ARM_NUM_IRQ_PRIORITY_BITS

#define DT_FLASH_DEV_BASE_ADDRESS		DT_SILABS_SIM3_FLASH_CONTROLLER_400C0000_BASE_ADDRESS
#define DT_FLASH_DEV_NAME			DT_SILABS_SIM3_FLASH_CONTROLLER_400C0000_LABEL

#define DT_GPIO_SIM3_COMMON_NAME	DT_SILABS_SIM3_GPIO_40002A00_LABEL

/* End of SoC Level DTS fixup file */
