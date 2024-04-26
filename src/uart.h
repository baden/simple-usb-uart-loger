#pragma once

#include <stdint.h>
#include <hardware/uart.h>
#include <hardware/irq.h>
// #include <pico/multicore.h>
#include <tusb.h>

#define ISR_BUFFER_SIZE 2560
#define LINE_BUFFER_SIZE 2560

#define DEF_BIT_RATE 115200
#define DEF_STOP_BITS 1
#define DEF_PARITY 0
#define DEF_DATA_BITS 8

typedef struct {
	uart_inst_t *const inst;
	uint irq;
	void *irq_fn;
	uint8_t tx_pin;
	uint8_t rx_pin;
} uart_id_t;

typedef struct {
	cdc_line_coding_t uart_lc;
	// mutex_t lc_mtx;
	uint8_t uart_buffer[ISR_BUFFER_SIZE];	// ISR buffer
	uint32_t uart_pos;
	uint32_t timestamp;

	// uint8_t line_buffer[LINE_BUFFER_SIZE];	// Line buffer
	// uint32_t line_pos;

	mutex_t uart_mtx;
	// uint32_t usb_pos;
	// mutex_t usb_mtx;
} uart_data_t;

void init_uart(); //_data(uint8_t itf);
void uart_task(void);
void uart_reset(void);
void uart_flush(void);
void uart_baudrate(uint32_t baudrate);
