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
#include <hardware/uart.h>
#include <hardware/structs/sio.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>

#include "bsp/board.h"
#include "tusb.h"
#include "usb_descriptors.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED     = 1000,
  BLINK_SUSPENDED   = 2500,

  BLINK_ALWAYS_ON   = UINT32_MAX,
  BLINK_ALWAYS_OFF  = 0
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

#define CURRENT_TIME_MS to_ms_since_boot(get_absolute_time())

#define URL  "example.tinyusb.org/webusb-serial/index.html"

const tusb_desc_webusb_url_t desc_url =
{
  .bLength         = 3 + sizeof(URL) - 1,
  .bDescriptorType = 3, // WEBUSB URL type
  .bScheme         = 1, // 0: http, 1: https
  .url             = URL
};

static bool web_serial_connected = false;

//------------- prototypes -------------//
void led_blinking_task(void);
void cdc_task(void);
void webserial_task(void);

#define UART0_TX_PIN 12
#define UART0_RX_PIN 13
#define UART1_TX_PIN 8
#define UART1_RX_PIN 9

void uart0_irq_fn(void);
void uart1_irq_fn(void);



void init_uart_data()
{
  /* Pinmux */
	gpio_set_function(UART0_TX_PIN, GPIO_FUNC_UART);
	gpio_set_pulls(UART0_TX_PIN, true, false);
	gpio_set_function(UART0_RX_PIN, GPIO_FUNC_UART);
	gpio_set_pulls(UART0_RX_PIN, true, false);

  /* UART start */
  uart_init(uart0, 115200);
  uart_set_hw_flow(uart0, false, false);
  uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
  uart_set_fifo_enabled(uart0, false);

  /* UART RX Interrupt */
  irq_set_exclusive_handler(UART0_IRQ, uart0_irq_fn);
  irq_set_enabled(UART0_IRQ, true);
  uart_set_irq_enables(uart0, true, false);

  // UART1
  /* Pinmux */
  gpio_set_function(UART1_TX_PIN, GPIO_FUNC_UART);
  gpio_set_pulls(UART1_TX_PIN, true, false);
  gpio_set_function(UART1_RX_PIN, GPIO_FUNC_UART);
  gpio_set_pulls(UART1_RX_PIN, true, false);

  /* UART start */
  uart_init(uart1, 115200);
  uart_set_hw_flow(uart1, false, false);
  uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
  uart_set_fifo_enabled(uart1, false);

  /* UART RX Interrupt */
  irq_set_exclusive_handler(UART1_IRQ, uart1_irq_fn);
  irq_set_enabled(UART1_IRQ, true);
  uart_set_irq_enables(uart1, true, false);

  // Enable UART IRQ
//  irq_set_enabled(UART0_IRQ, true);
//  irq_set_enabled(UART1_IRQ, true);

}


/*------------- MAIN -------------*/
int main(void)
{
  board_init();
  init_uart_data();

  // init device stack on configured roothub port
  tud_init(BOARD_TUD_RHPORT);

  while (1)
  {
    tud_task(); // tinyusb device task
    cdc_task();
    webserial_task();
    led_blinking_task();
  }

  return 0;
}

// send characters to both CDC and WebUSB
void echo_all(uint8_t buf[], uint32_t count)
{
  // echo to web serial
  if ( web_serial_connected )
  {
    tud_vendor_write(buf, count);
    tud_vendor_flush();
  }

  // echo to cdc
  if ( tud_cdc_connected() )
  {
    for(uint32_t i=0; i<count; i++)
    {
      tud_cdc_write_char(buf[i]);

      if ( buf[i] == '\r' ) tud_cdc_write_char('\n');
    }
    tud_cdc_write_flush();
  }

  // echo to uart
  uart_write_blocking(uart0, buf, count);
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

//--------------------------------------------------------------------+
// WebUSB use vendor class
//--------------------------------------------------------------------+

// Invoked when a control transfer occurred on an interface of this class
// Driver response accordingly to the request and the transfer stage (setup/data/ack)
// return false to stall control endpoint (e.g unsupported request)
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
  // nothing to with DATA & ACK stage
  if (stage != CONTROL_STAGE_SETUP) return true;

  switch (request->bmRequestType_bit.type)
  {
    case TUSB_REQ_TYPE_VENDOR:
      switch (request->bRequest)
      {
        case VENDOR_REQUEST_WEBUSB:
          // match vendor request in BOS descriptor
          // Get landing page url
          return tud_control_xfer(rhport, request, (void*)(uintptr_t) &desc_url, desc_url.bLength);

        case VENDOR_REQUEST_MICROSOFT:
          if ( request->wIndex == 7 )
          {
            // Get Microsoft OS 2.0 compatible descriptor
            uint16_t total_len;
            memcpy(&total_len, desc_ms_os_20+8, 2);

            return tud_control_xfer(rhport, request, (void*)(uintptr_t) desc_ms_os_20, total_len);
          }else
          {
            return false;
          }

        default: break;
      }
    break;

    case TUSB_REQ_TYPE_CLASS:
      if (request->bRequest == 0x22)
      {
        // Webserial simulate the CDC_REQUEST_SET_CONTROL_LINE_STATE (0x22) to connect and disconnect.
        web_serial_connected = (request->wValue != 0);

        // Always lit LED if connected
        if ( web_serial_connected )
        {
          board_led_write(true);
          blink_interval_ms = BLINK_ALWAYS_ON;

          tud_vendor_write_str("\r\nWebUSB interface connected\r\n");
          tud_vendor_flush();
        }else
        {
          blink_interval_ms = BLINK_MOUNTED;
        }

        // response with status OK
        return tud_control_status(rhport, request);
      }
    break;

    default: break;
  }

  // stall unknown request
  return false;
}

void webserial_task(void)
{
  if ( web_serial_connected )
  {
    if ( tud_vendor_available() )
    {
      uint8_t buf[64];
      uint32_t count = tud_vendor_read(buf, sizeof(buf));

      // echo back to both web serial and cdc
      echo_all(buf, count);
    }
  }
}


//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
void cdc_task(void)
{
  if ( tud_cdc_connected() )
  {
    // connected and there are data available
    if ( tud_cdc_available() )
    {
      uint8_t buf[64];

      uint32_t count = tud_cdc_read(buf, sizeof(buf));

      // echo back to both web serial and cdc
      echo_all(buf, count);
    }
  }
}
#define CURRENT_TIME_MS to_ms_since_boot(get_absolute_time())
uint32_t buf0_start = 0;
uint8_t buf0[512];
unsigned int buf0_pos = 0;

uint32_t buf1_start = 0;
uint8_t buf1[512];
unsigned int buf1_pos = 0;

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

void uart0_irq_fn(void)
{
  // Get all available data from UART
  while ( uart_is_readable(uart0) )
  {
    // uint8_t buf[64];
    // size_t pos = 0;
    while (uart_is_readable(uart0)) {
      uint8_t c = uart_getc(uart0);
      if(c == '\n') continue;
      if(c == '\r') {
        if(buf0_pos > 0) {
          // echo back to both web serial and cdc
          char timestamp[32];
          snprintf(timestamp, sizeof(timestamp), ">[%d] : [", CURRENT_TIME_MS - buf0_start);
          // echo_all(red, sizeof(red) - 1);
          echo_all((uint8_t *)timestamp, strlen(timestamp));
          echo_all(buf0, buf0_pos);
          char newline[2] = {']', '\r'};
          echo_all(newline, 2);
          // echo_all(reset, sizeof(reset) - 1);
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

    // uint32_t count = uart_read_blocking(uart0, buf, sizeof(buf));

    // echo back to both web serial and cdc
    // echo_all(buf, pos);
  }
}

void uart1_irq_fn(void)
{
  // Get all available data from UART
  while ( uart_is_readable(uart1) )
  {
    uint8_t buf[64];
    size_t pos = 0;
    while (uart_is_readable(uart1) &&
           (pos < sizeof(buf))) {
      buf[pos++] = uart_getc(uart1);
    }

    // uint32_t count = uart_read_blocking(uart1, buf, sizeof(buf));

    // echo back to both web serial and cdc
    echo_all(buf, pos);
  }
}


// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  (void) itf;

  // connected
  if ( dtr && rts )
  {
    // print initial message when connected
    tud_cdc_write_str("\r\nTinyUSB WebUSB device example\r\n");
  }
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
  (void) itf;
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if ( board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}
