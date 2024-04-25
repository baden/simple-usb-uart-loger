#include "led.h"
#include <stdbool.h>
#include "bsp/board.h"

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

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
