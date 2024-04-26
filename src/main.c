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
#include "tusb.h"
#include "usb_descriptors.h"
#include "led.h"
#include "uart.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

int uart_baudrate = 115200;


#define CURRENT_TIME_MS to_ms_since_boot(get_absolute_time())

#define URL  "baden.github.io/simple-usb-uart-loger/index.html"

const tusb_desc_webusb_url_t desc_url =
{
  .bLength         = 3 + sizeof(URL) - 1,
  .bDescriptorType = 3, // WEBUSB URL type
  .bScheme         = 1, // 0: http, 1: https
  .url             = URL
};

static bool web_serial_connected = false;

//------------- prototypes -------------//
void cdc_task(void);
void webserial_task(void);

// #define UART0_TX_PIN 12
// #define UART0_RX_PIN 13
// #define UART1_TX_PIN 8
// #define UART1_RX_PIN 9

// void uart0_irq_fn(void);
// void uart1_irq_fn(void);



/*------------- MAIN -------------*/
int main(void)
{
  board_init();
  init_uart();

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

// // send characters to both CDC and WebUSB
// void echo_all(uint8_t buf[], uint32_t count)
// {
//   // echo to web serial
//   if ( web_serial_connected )
//   {
//     tud_vendor_write(buf, count);
//     tud_vendor_flush();
//   }

//   // echo to cdc
//   if ( tud_cdc_connected() )
//   {
//     for(uint32_t i=0; i<count; i++)
//     {
//       tud_cdc_write_char(buf[i]);

//       if ( buf[i] == '\r' ) tud_cdc_write_char('\n');
//     }
//     tud_cdc_write_flush();
//   }

//   // echo to uart
//   uart_write_blocking(uart0, buf, count);
// }

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  led_set_interval(BLINK_MOUNTED);
  // Send hello
  char buf[64];
  snprintf(buf, sizeof(buf), "Connected on %d\r\n", uart_baudrate);
  tud_vendor_write_str(buf);
  tud_vendor_flush();
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  led_set_interval(BLINK_NOT_MOUNTED);
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  led_set_interval(BLINK_SUSPENDED);
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  led_set_interval(BLINK_MOUNTED);
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
          led_set_interval(BLINK_ALWAYS_ON);

          tud_vendor_write_str("\r\nWebUSB interface connected\r\n");
          tud_vendor_flush();
        }else
        {
          // blink_interval_ms = BLINK_MOUNTED;
          led_set_interval(BLINK_MOUNTED);
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

void parse_cmd(uint8_t cmd);


void webserial_task(void)
{
  if ( web_serial_connected )
  {
    if ( tud_vendor_available() )
    {
      uint8_t buf[64];
      uint32_t count = tud_vendor_read(buf, sizeof(buf));
      for(uint32_t i=0; i<count; i++)
      {
        parse_cmd(buf[i]);
      }

      // echo back to both web serial and cdc
      // echo_all(buf, count);
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
      for(uint32_t i=0; i<count; i++)
      {
        parse_cmd(buf[i]);
      }

      // echo back to both web serial and cdc
      // echo_all(buf, count);
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

void parse_cmd(uint8_t cmd)
{
  switch (cmd)
  {
    case 'R':
      // Reset timer
      buf0_start = CURRENT_TIME_MS;
      buf0_pos = 0;
      buf1_start = CURRENT_TIME_MS;
      buf0_pos = 0;
      break;

    case 'F':
      // Flush buffers
      buf0_pos = 0;
      buf1_pos = 0;
      break;

    case '0':
      // Set UART0 & UART1 baudrates to 2400
      set_baudrate(2400);
      break;
    case '1':
      // Set UART0 & UART1 baudrates to 4800
      set_baudrate(4800);
      break;
    case '2':
      // Set UART0 & UART1 baudrates to 9600
      set_baudrate(9600);
      break;
    case '3':
      // Set UART0 & UART1 baudrates to 19200
      set_baudrate(19200);
      break;
    case '4':
      // Set UART0 & UART1 baudrates to 38400
      set_baudrate(38400);
      break;
    case '5':
      // Set UART0 & UART1 baudrates to 57600
      set_baudrate(57600);
      break;
    case '6':
      // Set UART0 & UART1 baudrates to 115200
      set_baudrate(115200);
      break;  
    case '7':
      // Set UART0 & UART1 baudrates to 230400
      set_baudrate(230400);
      break;
    case '8':
      // Set UART0 & UART1 baudrates to 460800
      set_baudrate(460800);
      break;
    case '9':
      // Set UART0 & UART1 baudrates to 921600
      set_baudrate(921600);
      break;

    default:
      break;
  }
  // uart_set_baudrate(uart0, bit_rate);
}


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

void uart0_irq_fn(void)
{
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
}

void uart1_irq_fn(void)
{
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
       
}


// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  (void) itf;

  // connected
  if ( tud_cdc_connected() && dtr && rts )
  {
    // print initial message when connected
    tud_cdc_write_str(
      "\r\n"
      "TinyUSB WebUSB device example\r\n"
      "[R]Reset timer    [F]Flush buffers   [0]2400 [1]4800 [2]9600 [3]19200 [4]38400 [5]57600 [6]115200 [7]230400 \r\n"
      "\r\n"
    );
    // tud_cdc_write_flush();
  }
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
  (void) itf;
}

