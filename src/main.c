/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

/* This example demonstrates WebUSB as web serial with browser with WebUSB support (e.g Chrome).
 * After enumerated successfully, browser will pop-up notification
 * with URL to landing page, click on it to test
 *  - Click "Connect" and select device, When connected the on-board LED will litted up.
 *  - Any charters received from either webusb/Serial will be echo back to webusb and Serial
 *
 * Note:
 * - The WebUSB landing page notification is currently disabled in Chrome
 * on Windows due to Chromium issue 656702 (https://crbug.com/656702). You have to
 * go to landing page (below) to test
 *
 * - On Windows 7 and prior: You need to use Zadig tool to manually bind the
 * WebUSB interface with the WinUSB driver for Chrome to access. From windows 8 and 10, this
 * is done automatically by firmware.
 *
 * - On Linux/macOS, udev permission may need to be updated by
 *   - copying '/examples/device/99-tinyusb.rules' file to /etc/udev/rules.d/ then
 *   - run 'sudo udevadm control --reload-rules && sudo udevadm trigger'
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <hardware/irq.h>
// #include <hardware/uart.h>
#include <hardware/structs/sio.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>

#include "bsp/board.h"
#include "led.h"
#include "uart.h"
#include "usb.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

// int uart_baudrate = 115200;

#define CURRENT_TIME_MS to_ms_since_boot(get_absolute_time())


/*------------- MAIN -------------*/
int main(void)
{
  board_init();
  init_uart();
  usb_init();
  led_init();

  while (1)
  {
    usb_task();
    // uart_task();
    led_blinking_task();
  }

  return 0;
}

#if 0
void set_baudrate(uint32_t baudrate)
{
  char buf[64];
  uart_baudrate = baudrate;
  uart_set_baudrate(uart0, uart_baudrate);
  uart_set_baudrate(uart1, uart_baudrate);
  snprintf(buf, sizeof(buf), "Baudrate: %d\r\n", uart_baudrate);
  if(tud_cdc_connected()) {
    tud_cdc_write_str(buf);
    tud_cdc_write_flush();
  }
  if(web_serial_connected) {
    tud_vendor_write_str(buf);
    tud_vendor_flush();
  }
  // if(tud_cdc_connected()) tud_cdc_write_str("Baudrate set to 115200\r\n");
}
#endif

#if 0
#define d2h(c) (((c) <= 9)?('0' + (c)):('A' + (c) - 10))

// TODO: Перенести це в відправку у відповідний порт, бо це треба тільки для CDC
// terminal colors
uint8_t red[] = "\x1b[31m";
uint8_t green[] = "\x1b[32m";
uint8_t yellow[] = "\x1b[33m";
uint8_t blue[] = "\x1b[34m";
uint8_t magenta[] = "\x1b[35m";
uint8_t cyan[] = "\x1b[36m";
uint8_t white[] = "\x1b[37m";
uint8_t reset[] = "\x1b[0m";

void uart_read(uart_inst_t *uart, uint8_t index)
{
}

void tud_cdc_write_line(const char *p, size_t len)
{
  for(uint32_t i=0; i<len; i++)
  {
    tud_cdc_write_char(p[i]);
    if ( p[i] == '\r' ) tud_cdc_write_char('\n');
  }
  // tud_cdc_write_flush();
}
#endif
