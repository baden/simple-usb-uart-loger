#pragma once

#include <stdint.h>
#include <stdlib.h>

#include "tusb.h"

void usb_init(void);
void usb_task(void);
void usb_push_line(unsigned index, uint8_t *data, size_t len, uint32_t timestamp);

extern cdc_line_coding_t usb_lc;
