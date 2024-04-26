// #include <pico/mutex.h>
#include <hardware/structs/sio.h>
#include <pico/stdlib.h>

#include "uart.h"

#define CURRENT_TIME_MS to_ms_since_boot(get_absolute_time())


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
	ud->line_pos = 0;

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

#define PICO_STDIO_USB_STDOUT_TIMEOUT_US 1000000

static void flush_line(uint8_t itf)
{
	uart_data_t *ud = &UART_DATA[itf];
	const uart_id_t *ui = &UART_ID[itf];
	static uint64_t last_avail_time[2];

	if (tud_vendor_mounted()) {
		// for(uint32_t i = 0; i < ud->line_pos; ) {
		uint32_t count = tud_vendor_write(ud->line_buffer, ud->line_pos);
		if (count < ud->uart_pos)
				memmove(ud->uart_buffer, &ud->uart_buffer[count],
					ud->uart_pos - count);
		ud->line_pos -= count;
		if(ud->line_pos) {
			tud_vendor_write(ud->line_buffer, ud->line_pos);
		}
		tud_vendor_flush();
	}

	#if 0
	if (tud_vendor_mounted()) {
		for(uint32_t i = 0; i < ud->line_pos; ) {
			uint32_t n = ud->line_pos - i;
			uint32_t avail = tud_vendor_write_available();
			if (n > avail) n = avail;
			if (n) {
				uint32_t n2 = tud_vendor_write(&ud->line_buffer[i], n);
				// thread_do_tud_task();	// ??? aah?
				tud_task(); // tinyusb device task
				i += n2;
				last_avail_time[itf] = time_us_64();
			} else {
				// thread_do_tud_task();	// ??? aah?
				tud_task(); // tinyusb device task
				if (!tud_vendor_mounted() ||
							(!tud_vendor_write_available() &&
							 time_us_64() > last_avail_time[itf] + PICO_STDIO_USB_STDOUT_TIMEOUT_US)) {
					break;
				}
			}
			// uart_write_blocking(ui->inst, &ud->line_buffer[i], 1);
			// tud_vendor_write(&ud->line_buffer[i], 1);
		}
	} else {
		last_avail_time[itf] = 0;
	}
	#endif
	ud->line_pos = 0;
}

static void uart_n_task(uint8_t itf)
{
	uart_data_t *ud = &UART_DATA[itf];
	const uart_id_t *ui = &UART_ID[itf];

	if (ud->uart_pos &&
		mutex_try_enter(&ud->uart_mtx, NULL)) {
		uint32_t count = 0;

		while(count < ud->uart_pos) {
			uint8_t c = ud->uart_buffer[count++];
			// Не дуже мені подобається це. Дуже це неефективно.
			// memmove(ud->uart_buffer, &ud->uart_buffer[1], ud->uart_pos - 1);

    		if(c == '\n') continue;
    		else if(c == '\r') {
				flush_line(itf);
				continue;
			}
			else {
				if(ud->line_pos < LINE_BUFFER_SIZE) {
					ud->line_buffer[ud->line_pos++] = c;
				}
			}


		}
		// // uart_write_blocking(ui->inst, ud->uart_buffer, ud->uart_pos);
		// count = tud_vendor_write(ud->uart_buffer, ud->uart_pos);
		if(count < ud->uart_pos)
			memmove(ud->uart_buffer, &ud->uart_buffer[count], ud->uart_pos - count);

		ud->uart_pos -= count;

		// ud->uart_pos -= count;
		mutex_exit(&ud->uart_mtx);
		// if (count)
		// 		tud_vendor_flush();
	}
}

void uart_task(void)
{
	uart_n_task(0);
	uart_n_task(1);
}

static inline void uart_read_bytes(uint8_t itf)
{
	uart_data_t *ud = &UART_DATA[itf];
	const uart_id_t *ui = &UART_ID[itf];

	if (uart_is_readable(ui->inst)) {
		mutex_enter_blocking(&ud->uart_mtx);

		while (uart_is_readable(ui->inst) &&
		       (ud->uart_pos < ISR_BUFFER_SIZE)) {
			ud->uart_buffer[ud->uart_pos] = uart_getc(ui->inst);
			ud->uart_pos++;
		}

		mutex_exit(&ud->uart_mtx);
	}
}


void uart0_irq_fn(void)
{
	uart_read_bytes(0);
#if 0
  // Get all available data from UART
  while (uart_is_readable(uart0)) {
    uint8_t c = uart_getc(uart0);
    if(c == '\n') continue;
    if(c == '\r') {
      if(buf0_pos > 0) {
        // echo back to both web serial and cdc
        uint16_t ts = CURRENT_TIME_MS - buf0_start;
        int hours = ts / 3600000;
        int minutes = (ts % 3600000) / 60000;
        int seconds = (ts % 60000) / 1000;
        int milliseconds = ts % 1000;
        char timestamp[64];
        size_t l = snprintf(timestamp, sizeof(timestamp),
          ">[%02d:%02d:%02d.%03d] : [", hours, minutes, seconds, milliseconds
        );
        // echo_all(red, sizeof(red) - 1);

        // echo_all((uint8_t *)timestamp, strlen(timestamp));
        // echo_all(buf0, buf0_pos);
        char newline[2] = {']', '\r'};
        // echo_all(newline, 2);
        // echo_all(reset, sizeof(reset) - 1);

        if(web_serial_connected)
        {
          tud_vendor_write((uint8_t *)timestamp, strlen(timestamp));
          tud_vendor_write(buf0, buf0_pos);
          tud_vendor_write(newline, 2);
          tud_vendor_flush();
        }

        if ( tud_cdc_connected() )
        {
          tud_cdc_write_line(red, sizeof(red) - 1);
          tud_cdc_write_line(timestamp, strlen(timestamp));
          tud_cdc_write_line((const char *)buf0, buf0_pos);
          tud_cdc_write_line(newline, 2);
          tud_cdc_write_line(reset, sizeof(reset) - 1);
          tud_cdc_write_flush();
        }


        buf0_pos = 0;
      }
      continue;
    }
    if(buf0_pos < sizeof(buf0)) {
      if(c < ' ' && buf0_pos < (sizeof(buf0) - 5)) {
        buf0[buf0_pos++] = '\\';
        buf0[buf0_pos++] = 'x';
        buf0[buf0_pos++] = d2h(c >> 4);
        buf0[buf0_pos++] = d2h(c & 0xf);
      }
      buf0[buf0_pos++] = c;
    }
  }
#endif
}

void uart1_irq_fn(void)
{
	uart_read_bytes(1);
#if 0
  // Get all available data from UART
  while (uart_is_readable(uart1)) {
    uint8_t c = uart_getc(uart1);
    if(c == '\n') continue;
    if(c == '\r') {
      if(buf1_pos > 0) {
        // echo back to both web serial and cdc
        uint16_t ts = CURRENT_TIME_MS - buf1_start;
        int hours = ts / 3600000;
        int minutes = (ts % 3600000) / 60000;
        int seconds = (ts % 60000) / 1000;
        int milliseconds = ts % 1000;
        char timestamp[64];
        size_t l = snprintf(timestamp, sizeof(timestamp),
          "<[%02d:%02d:%02d.%03d] : [", hours, minutes, seconds, milliseconds
        );
        // echo_all(red, sizeof(red) - 1);

        // echo_all((uint8_t *)timestamp, strlen(timestamp));
        // echo_all(buf1, buf1_pos);
        char newline[2] = {']', '\r'};
        // echo_all(newline, 2);
        // echo_all(reset, sizeof(reset) - 1);

        if(web_serial_connected)
        {
          tud_vendor_write((uint8_t *)timestamp, strlen(timestamp));
          tud_vendor_write(buf1, buf1_pos);
          tud_vendor_write(newline, 2);
          tud_vendor_flush();
        }

        if ( tud_cdc_connected() )
        {
          tud_cdc_write_line(green, sizeof(red) - 1);
          tud_cdc_write_line(timestamp, strlen(timestamp));
          tud_cdc_write_line((const char *)buf1, buf1_pos);
          tud_cdc_write_line(newline, 2);
          tud_cdc_write_line(reset, sizeof(reset) - 1);
          tud_cdc_write_flush();
        }


        buf1_pos = 0;
      }
      continue;
    }
    if(buf1_pos < sizeof(buf1)) {
      if(c < ' ' && buf1_pos < (sizeof(buf1) - 5)) {
        buf1[buf1_pos++] = '\\';
        buf1[buf1_pos++] = 'x';
        buf1[buf1_pos++] = d2h(c >> 4);
        buf1[buf1_pos++] = d2h(c & 0xf);
      }
      buf1[buf1_pos++] = c;
    }
  }
#endif
}

void uart_reset()
{

}

void uart_flush()
{

}
