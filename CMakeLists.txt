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
	src/usb_descriptors.c
	${PICO_SDK_PATH}/lib/tinyusb/hw/bsp/rp2040/family.c
)

target_compile_definitions(simple_usb_uart_loger PRIVATE
	FAMILY=rp2040
)

# target_compile_definitions(simple_usb_uart_loger PRIVATE
# 	PICO_DEFAULT_UART_TX_PIN=12
# 	PICO_DEFAULT_UART_RX_PIN=13
# )

target_include_directories(simple_usb_uart_loger PUBLIC
	./src/
	${PICO_SDK_PATH}/lib/tinyusb/src
	)

target_link_libraries(simple_usb_uart_loger
	hardware_flash
	pico_multicore
	pico_stdlib
	tinyusb_device)

pico_add_extra_outputs(simple_usb_uart_loger)

# family_configure_device_example(simple_usb_uart_loger noos)
