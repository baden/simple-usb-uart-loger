# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.13)

include(../pico-uart-bridge/pico-sdk/pico_sdk_init.cmake)

include(../pico-uart-bridge/pico-sdk/lib/tinyusb/hw/bsp/family_support.cmake)

project(webusb_serial)

pico_sdk_init()


add_executable(webusb_serial src/main.c src/usb_descriptors.c
../pico-uart-bridge/pico-sdk/lib/tinyusb/hw/bsp/rp2040/family.c
)

# target_compile_definitions(webusb_serial PRIVATE
# 	PICO_DEFAULT_UART_TX_PIN=12
# 	PICO_DEFAULT_UART_RX_PIN=13
# )

target_include_directories(webusb_serial PUBLIC
	./src/
	../pico-uart-bridge/pico-sdk/lib/tinyusb/src
	)

target_link_libraries(webusb_serial
	hardware_flash
	pico_multicore
	pico_stdlib
	tinyusb_device)

pico_add_extra_outputs(webusb_serial)

# family_configure_device_example(webusb_serial noos)
