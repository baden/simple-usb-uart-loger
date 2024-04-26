// #include <pico/mutex.h>
#include <hardware/structs/sio.h>
#include <pico/stdlib.h>

#include "uart.h"


#define DEF_BIT_RATE 115200
#define DEF_STOP_BITS 1
#define DEF_PARITY 0
#define DEF_DATA_BITS 8

void uart0_irq_fn(void);
void uart1_irq_fn(void);

const uart_id_t UART_ID[2] = {
	{
		.inst = uart0,
		.irq = UART0_IRQ,
		.irq_fn = &uart0_irq_fn,
		.tx_pin = 12,
		.rx_pin = 13,
	}, {
		.inst = uart1,
		.irq = UART1_IRQ,
		.irq_fn = &uart1_irq_fn,
		.tx_pin = 8,
		.rx_pin = 9,
	}
};

uart_data_t UART_DATA[2];

cdc_line_coding_t usb_lc;

static inline uint databits_usb2uart(uint8_t data_bits)
{
	switch (data_bits) {
		case 5:
			return 5;
		case 6:
			return 6;
		case 7:
			return 7;
		default:
			return 8;
	}
}

static inline uart_parity_t parity_usb2uart(uint8_t usb_parity)
{
	switch (usb_parity) {
		case 1:
			return UART_PARITY_ODD;
		case 2:
			return UART_PARITY_EVEN;
		default:
			return UART_PARITY_NONE;
	}
}

static inline uint stopbits_usb2uart(uint8_t stop_bits)
{
	switch (stop_bits) {
		case 2:
			return 2;
		default:
			return 1;
	}
}

static void init_uart_data(uint8_t itf)
{
	const uart_id_t *ui = &UART_ID[itf];
	uart_data_t *ud = &UART_DATA[itf];


  /* Pinmux */
	gpio_set_function(ui->tx_pin, GPIO_FUNC_UART);
	gpio_set_pulls(ui->tx_pin, true, false);
	gpio_set_function(ui->rx_pin, GPIO_FUNC_UART);
	gpio_set_pulls(ui->rx_pin, true, false);

	/* UART LC */
	ud->uart_lc.bit_rate = usb_lc.bit_rate;
	ud->uart_lc.data_bits = usb_lc.data_bits;
	ud->uart_lc.parity = usb_lc.parity;
	ud->uart_lc.stop_bits = usb_lc.stop_bits;

	/* Buffer */
	ud->uart_pos = 0;
	// ud->usb_pos = 0;

	/* Mutex */
	mutex_init(&ud->lc_mtx);
	mutex_init(&ud->uart_mtx);
	// mutex_init(&ud->usb_mtx);

    /* UART start */
    uart_init(ui->inst, usb_lc.bit_rate);
    uart_set_hw_flow(ui->inst, false, false);
    uart_set_format(ui->inst, 8, 1, UART_PARITY_NONE);
    // Якшо треба буде змінювати налаштування по USB-CDC:
	// uart_set_format(ui->inst, databits_usb2uart(usb_lc.data_bits),
	// 		stopbits_usb2uart(usb_lc.stop_bits),
	// 		parity_usb2uart(usb_lc.parity));

    uart_set_fifo_enabled(ui->inst, false);

    /* UART RX Interrupt */
    irq_set_exclusive_handler(ui->irq, ui->irq_fn);
    irq_set_enabled(ui->irq, true);
    uart_set_irq_enables(ui->inst, true, false);

    // Enable UART IRQ (не треба? а?)
    //  irq_set_enabled(ui->irq, true);
}

void init_uart()
{
    /* USB CDC LC  (TODO: Move to USB lib?)*/
	usb_lc.bit_rate = DEF_BIT_RATE;
	usb_lc.data_bits = DEF_DATA_BITS;
	usb_lc.parity = DEF_PARITY;
	usb_lc.stop_bits = DEF_STOP_BITS;

    init_uart_data(0);
    init_uart_data(1);
}


void uart_task(void)
{

}