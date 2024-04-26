# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/pico-sdk)
set(FAMILY rp2040)

include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

include(${PICO_SDK_PATH}/lib/tinyusb/hw/bsp/family_support.cmake)

project(simple_usb_uart_loger)

pico_sdk_init()

add_executable(simple_usb_uart_loger
	src/main.c
	src/led.c
	src/uart.c
	src/usb_descriptors.c
	pio-ws2812-rp2040/sdk/ws2812.c
	${PICO_SDK_PATH}/lib/tinyusb/hw/bsp/rp2040/family.c
)

pico_generate_pio_header(${PROJECT_NAME}
	${CMAKE_CURRENT_LIST_DIR}/pio-ws2812-rp2040/sdk/ws2812.pio
)

target_compile_definitions(simple_usb_uart_loger PRIVATE
	FAMILY=rp2040
)

target_compile_definitions(simple_usb_uart_loger PRIVATE
# 	PICO_DEFAULT_UART_TX_PIN=12
# 	PICO_DEFAULT_UART_RX_PIN=13
	# PICO_DEFAULT_LED_PIN=11
	PICO_DEFAULT_WS2812_PIN=16
)

target_include_directories(simple_usb_uart_loger PUBLIC
	./src/
	${PICO_SDK_PATH}/lib/tinyusb/src
	./pio-ws2812-rp2040/sdk
	)

target_link_libraries(simple_usb_uart_loger
	hardware_flash
	pico_multicore
	pico_stdlib
	hardware_pio
    hardware_clocks
	tinyusb_device)

pico_add_extra_outputs(simple_usb_uart_loger)

# family_configure_device_example(simple_usb_uart_loger noos)
