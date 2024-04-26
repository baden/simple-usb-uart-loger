#include "usb.h"
#include "uart.h"
#include "pico/mutex.h"
#include "led.h"
#include "usb_descriptors.h"

#define USB_TX_BUFFER_SIZE 16384


#define URL  "baden.github.io/simple-usb-uart-loger/index.html"

const tusb_desc_webusb_url_t desc_url =
{
  .bLength         = 3 + sizeof(URL) - 1,
  .bDescriptorType = 3, // WEBUSB URL type
  .bScheme         = 1, // 0: http, 1: https
  .url             = URL
};

static bool web_serial_connected = false;


static uint8_t usb_vendor_tx_buffer[USB_TX_BUFFER_SIZE];
static uint32_t usb_vendor_tx_pos = 0;
mutex_t usb_vendor_tx_mtx;

static uint8_t usb_cdc_tx_buffer[USB_TX_BUFFER_SIZE];
static uint32_t usb_cdc_tx_pos = 0;
static mutex_t usb_cdc_tx_mtx;
cdc_line_coding_t usb_lc;
static mutex_t lc_mtx;


void usb_init(void)
{
    // init device stack on configured roothub port
    tud_init(BOARD_TUD_RHPORT);

    mutex_init(&usb_vendor_tx_mtx);
    mutex_init(&usb_cdc_tx_mtx);

    mutex_init(&lc_mtx);
	usb_lc.bit_rate = DEF_BIT_RATE;
	usb_lc.data_bits = DEF_DATA_BITS;
	usb_lc.parity = DEF_PARITY;
	usb_lc.stop_bits = DEF_STOP_BITS;

}

void parse_cmd(uint8_t cmd);


//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
static void cdc_task(void)
{
  if ( tud_cdc_connected() )
  {
	// usb_cdc_process(0 /*itf*/);
	mutex_enter_blocking(&lc_mtx);
	tud_cdc_get_line_coding(&usb_lc);
	mutex_exit(&lc_mtx);

    // READ
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

static void webserial_task(void)
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

    // WRITE
    if(usb_vendor_tx_pos && 
        mutex_try_enter(&usb_vendor_tx_mtx, NULL)){
        uint32_t count = tud_vendor_write(usb_vendor_tx_buffer, usb_vendor_tx_pos);
        if(count < usb_vendor_tx_pos){
            memmove(usb_vendor_tx_buffer, usb_vendor_tx_buffer + count, usb_vendor_tx_pos - count);
        }
        usb_vendor_tx_pos -= count;
        mutex_exit(&usb_vendor_tx_mtx);
        if(count){
            tud_vendor_flush();
        }
    }


  }
}


void usb_task(void)
{
    tud_task(); // tinyusb device task
    cdc_task();
    webserial_task();
}


void usb_push_line(unsigned index, uint8_t *data, size_t len, uint32_t ts)
{
    mutex_enter_blocking(&usb_vendor_tx_mtx);
    if(index < 3) {
        uint8_t buf[64];
        int hours = ts / 3600000;
        int minutes = (ts % 3600000) / 60000;
        int seconds = (ts % 60000) / 1000;
        int milliseconds = ts % 1000;
        size_t l = snprintf(buf, sizeof(buf),
            "%c[%02d:%02d:%02d.%03d]:",
            index?'>':'<',
            hours, minutes, seconds, milliseconds
        );
        size_t len0 = MIN(l, USB_TX_BUFFER_SIZE - usb_vendor_tx_pos);
        memcpy(usb_vendor_tx_buffer + usb_vendor_tx_pos, buf, len0);
        usb_vendor_tx_pos += len0;
    }
    

    size_t len1 = MIN(len, USB_TX_BUFFER_SIZE - usb_vendor_tx_pos);
    memcpy(usb_vendor_tx_buffer + usb_vendor_tx_pos, data, len1);
    usb_vendor_tx_pos += len1;
    // if (usb_vendor_tx_pos + len < USB_TX_BUFFER_SIZE)
    // {
    //     memcpy(usb_vendor_tx_buffer + usb_vendor_tx_pos, data, len);
    //     usb_vendor_tx_pos += len;
    // }
    mutex_exit(&usb_vendor_tx_mtx);
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  led_set_interval(BLINK_MOUNTED);
  // Send hello
  char buf[64];
  snprintf(buf, sizeof(buf), "Connected on %d\r\n", 0/*uart_baudrate*/);
  usb_push_line(3, buf, strlen(buf)-1, 0);

//   tud_vendor_write_str(buf);
//   tud_vendor_flush();
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

        //   tud_vendor_write_str("\r\nWebUSB interface connected\r\n");
        //   tud_vendor_flush();
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


void parse_cmd(uint8_t cmd)
{
  switch (cmd)
  {
    case 'R':
      // Reset timer
      // buf0_start = CURRENT_TIME_MS;
      // buf0_pos = 0;
      // buf1_start = CURRENT_TIME_MS;
      // buf0_pos = 0;
      uart_reset();
      break;

    case 'F':
      // Flush buffers
      // buf0_pos = 0;
      // buf1_pos = 0;
      uart_flush();
      break;

    case '0':
      // Set UART0 & UART1 baudrates to 2400
      uart_baudrate(2400);
      break;
    case '1':
      // Set UART0 & UART1 baudrates to 4800
      uart_baudrate(4800);
      break;
    case '2':
      // Set UART0 & UART1 baudrates to 9600
      uart_baudrate(9600);
      break;
    case '3':
      // Set UART0 & UART1 baudrates to 19200
      uart_baudrate(19200);
      break;
    case '4':
      // Set UART0 & UART1 baudrates to 38400
      uart_baudrate(38400);
      break;
    case '5':
      // Set UART0 & UART1 baudrates to 57600
      uart_baudrate(57600);
      break;
    case '6':
      // Set UART0 & UART1 baudrates to 115200
      uart_baudrate(115200);
      break;  
    case '7':
      // Set UART0 & UART1 baudrates to 230400
      uart_baudrate(230400);
      break;
    case '8':
      // Set UART0 & UART1 baudrates to 460800
      uart_baudrate(460800);
      break;
    case '9':
      // Set UART0 & UART1 baudrates to 921600
      uart_baudrate(921600);
      break;

    default:
      break;
  }
  // uart_set_baudrate(uart0, bit_rate);
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


// // send characters to both CDC and WebUSB and UART?
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

