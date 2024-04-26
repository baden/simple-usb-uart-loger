#include "led.h"
#include <stdbool.h>
#include "bsp/board.h"
#include "ws2812.h"

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

PIO pio = pio0; // values: pio0, pio1
const uint WS2812_PIN = PICO_DEFAULT_WS2812_PIN; //16

void led_set_interval(uint32_t interval_ms)
{
  blink_interval_ms = interval_ms;
  if(interval_ms == BLINK_ALWAYS_ON) {
    board_led_write(true);
  }
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

#include <stdlib.h>
int64_t alarm_callback(alarm_id_t id, void *user_data)
{
    uint8_t red, green, blue;
    red = rand();
    green = rand();
    blue = rand();
    put_pixel_rgb(red / 4, green / 4, blue / 4);
    return 500 * 1000;
}

void led_init(void)
{
    ws2812_init(pio, WS2812_PIN, 800000);
    add_alarm_in_ms(500, alarm_callback, NULL, false);

}