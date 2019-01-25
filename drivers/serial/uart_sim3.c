/*
 * Copyright (c) 2019, Christian Taedcke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <uart.h>
#include <soc.h>

struct uart_sim3_config {
	UART_Type *base;
	u32_t baud_rate;
    //struct soc_gpio_pin pin_rx;
    //struct soc_gpio_pin pin_tx;
    //unsigned int loc;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	void (*irq_config_func)(struct device *dev);
#endif
};

struct uart_sim3_data {
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t callback;
	void *cb_data;
#endif
};

static int uart_sim3_poll_in(struct device *dev, unsigned char *c)
{
	const struct uart_sim3_config *config = dev->config->config_info;
	u8_t fifo_count = config->base->FIFOCN.bit.RCNT;

	if (fifo_count) {
	    *c = config->base->DATA.U8;
		return 0;
	}

	return -1;
}

static void uart_sim3_poll_out(struct device *dev, unsigned char c)
{
	const struct uart_sim3_config *config = dev->config->config_info;

	/* Wait for transmitter fifo to be empty */
	while (config->base->FIFOCN.bit.TCNT)
		;


	config->base->DATA.U8 = c;
}

static int uart_sim3_err_check(struct device *dev)
{
	const struct uart_sim3_config *config = dev->config->config_info;
	u32_t flags = config->base->CONTROL.reg;
	int err = 0;

	if (flags & UART_CONTROL_ROREI_Msk) {
		err |= UART_ERROR_OVERRUN;
	}

	if (flags & UART_CONTROL_RPARERI_Msk) {
		err |= UART_ERROR_PARITY;
	}

	if (flags & UART_CONTROL_RFRMERI_Msk) {
		err |= UART_ERROR_FRAMING;
	}
	
	config->base->CONTROL_CLR = UART_CONTROL_RFRMERI_Msk | UART_CONTROL_RPARERI_Msk | UART_CONTROL_ROREI_Msk;

	return err;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int uart_sim3_fifo_fill(struct device *dev, const u8_t *tx_data,
			       int len)
{
	const struct uart_sim3_config *config = dev->config->config_info;
	u8_t num_tx = 0U;

	while ((len - num_tx > 0) && (config->base->FIFOCN.bit.TCNT == 0)) {
		config->base->DATA.U8 = tx_data[num_tx++];
	}

	return num_tx;
}

static int uart_sim3_fifo_read(struct device *dev, u8_t *rx_data,
			       const int len)
{
	const struct uart_sim3_config *config = dev->config->config_info;
	u8_t num_rx = 0U;

	while ((len - num_rx > 0) &&
	       (num_rx < config->base->FIFOCN.bit.RCNT)) {

		rx_data[num_rx++] = config->base->DATA.U8;
	}

	return num_rx;
}

static void uart_sim3_irq_tx_enable(struct device *dev)
{
	const struct uart_sim3_config *config = dev->config->config_info;

        /* Enable the transmit complete interrupt */
	config->base->CONTROL_SET = UART_CONTROL_TCPTIEN_Msk | UART_CONTROL_TDREQIEN_Msk;
}

static void uart_sim3_irq_tx_disable(struct device *dev)
{
	const struct uart_sim3_config *config = dev->config->config_info;

	config->base->CONTROL_CLR = UART_CONTROL_TCPTIEN_Msk | UART_CONTROL_TDREQIEN_Msk;
}

static int uart_sim3_irq_tx_complete(struct device *dev)
{
	const struct uart_sim3_config *config = dev->config->config_info;
	u32_t flags = config->base->CONTROL.reg;

	config->base->CONTROL_CLR = UART_CONTROL_TCPTI_Msk;

	return (flags & UART_CONTROL_TCPTI_Msk) != 0;
}

static int uart_sim3_irq_tx_ready(struct device *dev)
{
	const struct uart_sim3_config *config = dev->config->config_info;
	u32_t flags = config->base->CONTROL.reg;

	config->base->CONTROL_CLR = UART_CONTROL_TDREQI_Msk;

	return (flags & UART_CONTROL_TDREQI_Msk) != 0;
}

static void uart_sim3_irq_rx_enable(struct device *dev)
{
	const struct uart_sim3_config *config = dev->config->config_info;

	config->base->CONTROL_SET = UART_CONTROL_RDREQIEN_Msk;
}

static void uart_sim3_irq_rx_disable(struct device *dev)
{
	const struct uart_sim3_config *config = dev->config->config_info;

	config->base->CONTROL_CLR = UART_CONTROL_RDREQIEN_Msk;
}

static int uart_sim3_irq_rx_full(struct device *dev)
{
	const struct uart_sim3_config *config = dev->config->config_info;
	int flag = config->base->CONTROL.bit.RDREQI;

	config->base->CONTROL_CLR = UART_CONTROL_RDREQI_Msk;
	
	return flag;
}

static int uart_sim3_irq_rx_ready(struct device *dev)
{
	const struct uart_sim3_config *config = dev->config->config_info;

	return config->base->CONTROL.bit.RDREQIEN && uart_sim3_irq_rx_full(dev);
}

static void uart_sim3_irq_err_enable(struct device *dev)
{
	const struct uart_sim3_config *config = dev->config->config_info;

	config->base->CONTROL_SET = UART_CONTROL_RERIEN_Msk;
}

static void uart_sim3_irq_err_disable(struct device *dev)
{
	const struct uart_sim3_config *config = dev->config->config_info;

	config->base->CONTROL_CLR = UART_CONTROL_RERIEN_Msk;
}

static int uart_sim3_irq_is_pending(struct device *dev)
{
	return uart_sim3_irq_tx_ready(dev) || uart_sim3_irq_rx_ready(dev);
}

static int uart_sim3_irq_update(struct device *dev)
{
	return 1;
}

static void uart_sim3_irq_callback_set(struct device *dev,
				       uart_irq_callback_user_data_t cb,
				       void *cb_data)
{
	struct uart_sim3_data *data = dev->driver_data;

	data->callback = cb;
	data->cb_data = cb_data;
}

static void uart_sim3_isr(void *arg)
{
	struct device *dev = arg;
	struct uart_sim3_data *data = dev->driver_data;

	if (data->callback) {
		data->callback(data->cb_data);
	}
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static void uart_sim3_init_pins(struct device *dev)
{
    //const struct uart_sim3_config *config = dev->config->config_info;

	CLKCTRL_0->APBCLKG0.bit.PLL0CEN = 1;
	CLKCTRL_0->APBCLKG0.bit.PB0CEN = 1;

	//PB1 is on XBAR 0
	PBCFG_0->XBAR0H.bit.XBAR0EN = 1;

	//skip all on PB0
	PBSTD_0->PBSKIPEN_SET = PBSTD_PBSKIPEN_PBSKIPEN_Msk;

        //skip all on PB1
	PBSTD_1->PBSKIPEN_SET = PBSTD_PBSKIPEN_PBSKIPEN_Msk;
	
	// PB1.12 RX CP210X
	// PB1.13 TX CP210X
	PBSTD_1->PBSKIPEN_CLR = (1U << 12) | (1U << 13);
	
	// PB1.14 CTS CP210X
	// PB1.15 RTS CP210X
	//do not use rts and cts for now

	u8_t pin = 13;
	//Configure PB1.13 as digital input
	PBSTD_1->PBOUTMD_CLR = (1U << pin); //recommended for input mode
	PBSTD_1->PB_SET      = (1U << pin); //recommended for input mode
	PBSTD_1->PBMDSEL_SET = (1U << pin); //set digital mode

	pin = 12;
	//Configure PB1.12 as digital output
	//PBSTD_1->PB_CLR      = (1U << pin); //set to 0
	PBSTD_1->PBOUTMD_SET = (1U << pin); //push-pull
	PBSTD_1->PBMDSEL_SET = (1U << pin); //digital mode

	//enable UART0EN in xbar0
	PBCFG_0->XBAR0H_SET = PBCFG_0_XBAR0H_UART0EN_Msk;
}

//#if defined(ENABLE_FAST_CLOCK) && ENABLE_FAST_CLOCK
//#define N 4 // 80 MHz
//#else
//#define N 2 // 20 MHz
//#endif
#define N (2)
#define CALC_BAUDRATE(baudrate)					\
    (u32_t)((u32_t)(SystemCoreClock / (N * (u32_t)baudrate)) -1 )

 //#define CALC_BAUDRATE(baudrate)				\
 //   (u32_t)((u32_t)(20000000 / (N * (u32_t)baudrate)) -1 )

static int uart_sim3_init(struct device *dev)
{
	const struct uart_sim3_config *config = dev->config->config_info;
	u16_t baud = CALC_BAUDRATE(config->baud_rate);

	/* The peripheral and gpio clock are already enabled from soc and gpio
	 * driver
	 */

	/* Enable UART clock */
	CLKCTRL_0->APBCLKG0.bit.UART1CEN = 1;
	CLKCTRL_0->APBCLKG0.bit.UART0CEN = 1;


	config->base->BAUDRATE.bit.TBAUD = baud;
	config->base->BAUDRATE.bit.RBAUD = baud;

	// 8n1 is reset value

	
	
	/* Initialize UART pins */
	uart_sim3_init_pins(dev);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->irq_config_func(dev);
#endif

	/* Enable RX and TX */
	config->base->CONTROL_SET = UART_CONTROL_REN_Msk | UART_CONTROL_TEN_Msk;
	return 0;
}

static const struct uart_driver_api uart_sim3_driver_api = {
	.poll_in = uart_sim3_poll_in,
	.poll_out = uart_sim3_poll_out,
	.err_check = uart_sim3_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_sim3_fifo_fill,
	.fifo_read = uart_sim3_fifo_read,
	.irq_tx_enable = uart_sim3_irq_tx_enable,
	.irq_tx_disable = uart_sim3_irq_tx_disable,
	.irq_tx_complete = uart_sim3_irq_tx_complete,
	.irq_tx_ready = uart_sim3_irq_tx_ready,
	.irq_rx_enable = uart_sim3_irq_rx_enable,
	.irq_rx_disable = uart_sim3_irq_rx_disable,
	.irq_rx_ready = uart_sim3_irq_rx_ready,
	.irq_err_enable = uart_sim3_irq_err_enable,
	.irq_err_disable = uart_sim3_irq_err_disable,
	.irq_is_pending = uart_sim3_irq_is_pending,
	.irq_update = uart_sim3_irq_update,
	.irq_callback_set = uart_sim3_irq_callback_set,
#endif
};

#ifdef CONFIG_UART_SIM3_0

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_sim3_config_func_0(struct device *dev);
#endif

static const struct uart_sim3_config uart_sim3_0_config = {
	.base = (UART_Type *)DT_SILABS_SIM3_UART_UART_0_BASE_ADDRESS,
	//.clock = cmuClock_UART0,
	.baud_rate = DT_SILABS_SIM3_UART_UART_0_CURRENT_SPEED,
	//.pin_rx = PIN_UART0_RXD,
	//.pin_tx = PIN_UART0_TXD,
	//.loc = DT_SILABS_SIM3_UART_UART_0_LOCATION,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = uart_sim3_config_func_0,
#endif
};

static struct uart_sim3_data uart_sim3_0_data;

DEVICE_AND_API_INIT(uart_0, DT_SILABS_SIM3_UART_UART_0_LABEL, &uart_sim3_init,
		    &uart_sim3_0_data, &uart_sim3_0_config, PRE_KERNEL_1,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &uart_sim3_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_sim3_config_func_0(struct device *dev)
{
	IRQ_CONNECT(DT_SILABS_SIM3_UART_UART_0_IRQ,
		    DT_SILABS_SIM3_UART_UART_0_IRQ_PRIORITY, uart_sim3_isr,
		    DEVICE_GET(uart_0), 0);

	irq_enable(DT_SILABS_SIM3_UART_UART_0_IRQ);
}
#endif

#endif /* CONFIG_UART_SIM3_0 */

#ifdef CONFIG_UART_SIM3_1

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_sim3_config_func_1(struct device *dev);
#endif

static const struct uart_sim3_config uart_sim3_1_config = {
	.base = (USART_TypeDef *)DT_SILABS_SIM3_UART_UART_1_BASE_ADDRESS,
	.clock = cmuClock_UART1,
	.baud_rate = DT_SILABS_SIM3_UART_UART_1_CURRENT_SPEED,
	.pin_rx = PIN_UART1_RXD,
	.pin_tx = PIN_UART1_TXD,
	.loc = DT_SILABS_SIM3_UART_UART_1_LOCATION,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = uart_sim3_config_func_1,
#endif
};

static struct uart_sim3_data uart_sim3_1_data;

DEVICE_AND_API_INIT(uart_1, DT_SILABS_SIM3_UART_UART_1_LABEL, &uart_sim3_init,
		    &uart_sim3_1_data, &uart_sim3_1_config, PRE_KERNEL_1,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &uart_sim3_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_sim3_config_func_1(struct device *dev)
{
	IRQ_CONNECT(DT_SILABS_SIM3_UART_UART_1_IRQ,
		    DT_SILABS_SIM3_UART_UART_1_IRQ_PRIORITY, uart_sim3_isr,
		    DEVICE_GET(uart_1), 0);

	irq_enable(DT_SILABS_SIM3_UART_UART_1_IRQ);
}
#endif

#endif /* CONFIG_UART_SIM3_1 */

#ifdef CONFIG_USART_SIM3_0

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void usart_sim3_config_func_0(struct device *dev);
#endif

static const struct uart_sim3_config usart_sim3_0_config = {
	.base = (USART_TypeDef *)DT_SILABS_SIM3_USART_USART_0_BASE_ADDRESS,
	.clock = cmuClock_USART0,
	.baud_rate = DT_SILABS_SIM3_USART_USART_0_CURRENT_SPEED,
	.pin_rx = PIN_USART0_RXD,
	.pin_tx = PIN_USART0_TXD,
	.loc = DT_SILABS_SIM3_USART_USART_0_LOCATION,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = usart_sim3_config_func_0,
#endif
};

static struct uart_sim3_data usart_sim3_0_data;

DEVICE_AND_API_INIT(usart_0, DT_SILABS_SIM3_USART_USART_0_LABEL,
		    &uart_sim3_init, &usart_sim3_0_data,
		    &usart_sim3_0_config, PRE_KERNEL_1,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &uart_sim3_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void usart_sim3_config_func_0(struct device *dev)
{
	IRQ_CONNECT(DT_SILABS_SIM3_USART_USART_0_IRQ_RX,
		    DT_SILABS_SIM3_USART_USART_0_IRQ_RX_PRIORITY,
		    uart_sim3_isr, DEVICE_GET(usart_0), 0);
	IRQ_CONNECT(DT_SILABS_SIM3_USART_USART_0_IRQ_TX,
		    DT_SILABS_SIM3_USART_USART_0_IRQ_TX_PRIORITY,
		    uart_sim3_isr, DEVICE_GET(usart_0), 0);

	irq_enable(DT_SILABS_SIM3_USART_USART_0_IRQ_RX);
	irq_enable(DT_SILABS_SIM3_USART_USART_0_IRQ_TX);
}
#endif

#endif /* CONFIG_USART_SIM3_0 */

#ifdef CONFIG_USART_SIM3_1

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void usart_sim3_config_func_1(struct device *dev);
#endif

static const struct uart_sim3_config usart_sim3_1_config = {
	.base = (USART_TypeDef *)DT_SILABS_SIM3_USART_USART_1_BASE_ADDRESS,
	.clock = cmuClock_USART1,
	.baud_rate = DT_SILABS_SIM3_USART_USART_1_CURRENT_SPEED,
	.pin_rx = PIN_USART1_RXD,
	.pin_tx = PIN_USART1_TXD,
	.loc = DT_SILABS_SIM3_USART_USART_1_LOCATION,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = usart_sim3_config_func_1,
#endif
};

static struct uart_sim3_data usart_sim3_1_data;

DEVICE_AND_API_INIT(usart_1, DT_SILABS_SIM3_USART_USART_1_LABEL,
		    &uart_sim3_init, &usart_sim3_1_data,
		    &usart_sim3_1_config, PRE_KERNEL_1,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &uart_sim3_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void usart_sim3_config_func_1(struct device *dev)
{
	IRQ_CONNECT(DT_SILABS_SIM3_USART_USART_1_IRQ_RX,
		    DT_SILABS_SIM3_USART_USART_1_IRQ_RX_PRIORITY,
		    uart_sim3_isr, DEVICE_GET(usart_1), 0);
	IRQ_CONNECT(DT_SILABS_SIM3_USART_USART_1_IRQ_TX,
		    DT_SILABS_SIM3_USART_USART_1_IRQ_TX_PRIORITY,
		    uart_sim3_isr, DEVICE_GET(usart_1), 0);

	irq_enable(DT_SILABS_SIM3_USART_USART_1_IRQ_RX);
	irq_enable(DT_SILABS_SIM3_USART_USART_1_IRQ_TX);
}
#endif

#endif /* CONFIG_USART_SIM3_1 */
