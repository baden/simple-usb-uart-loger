#pragma once

#include <stdint.h>
#include <hardware/uart.h>
#include <hardware/irq.h>
// #include <pico/multicore.h>
#include <tusb.h>

#define BUFFER_SIZE 2560

typedef struct {
	uart_inst_t *const inst;
	uint irq;
	void *irq_fn;
	uint8_t tx_pin;
	uint8_t rx_pin;
} uart_id_t;

typedef struct {
	cdc_line_coding_t uart_lc;
	mutex_t lc_mtx;
	uint8_t uart_buffer[BUFFER_SIZE];
	uint32_t uart_pos;
	mutex_t uart_mtx;
	// uint8_t usb_buffer[BUFFER_SIZE];
	// uint32_t usb_pos;
	// mutex_t usb_mtx;
} uart_data_t;

void init_uart(); //_data(uint8_t itf);
void uart_task(void);
