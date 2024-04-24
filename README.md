# Ще одна з ідей реалізації мультіпортового логера.

Обрати найдешевший процесор, який би мав USB і було б готове рішення для CDC-UART.


## Ізі-план.

Беремо це:
https://github.com/hathach/tinyusb/tree/master/examples/device/webusb_serial

І перероблюємо у робочу версію для того шо мені треба.

Перше шо спало на думку - взяти Raspberry RP2040-Zero

https://www.waveshare.com/wiki/RP2040-Zero

![схема](https://www.waveshare.com/w/upload/2/2b/RP2040-Zero-details-7.jpg)

### Підключаємо

| RP2040 GPIO | Function |
|:-----------:|:--------:|
|   GPIO12    | UART0 TX |
|   GPIO13    | UART0 RX |
|   GPIO8     | UART1 TX |
|   GPIO9     | UART1 RX |

