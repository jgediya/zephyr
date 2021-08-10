/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/pinctrl.h>

#include <hal/nrf_gpio.h>

#if DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_uart)
#define NRF_UART_TYPE NRF_UART_Type
#define NRF_UART_PSEL(uart, line) uart->PSEL##line
#elif DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_uarte)
#define NRF_UART_TYPE NRF_UARTE_Type
#define NRF_UART_PSEL(uart, line) uart->PSEL.line
#endif

/**
 * @brief Configure pin settings.
 *
 * @param pin Pin configuration.
 * @param dir Pin direction.
 * @param input Pin input buffer connection.
 */
static void nrf_pin_configure(pinctrl_soc_pin_t pin, nrf_gpio_pin_dir_t dir,
			      nrf_gpio_pin_input_t input)
{
	/* force input direction and disconnected buffer for low power */
	if (NRF_GET_LP(pin) == NRF_LP_ENABLE) {
		dir = NRF_GPIO_PIN_DIR_INPUT;
		input = NRF_GPIO_PIN_INPUT_DISCONNECT;
	}

	nrf_gpio_cfg(NRF_GET_PIN(pin), dir, input, NRF_GET_PULL(pin),
		     NRF_GET_DRIVE(pin), NRF_GPIO_PIN_NOSENSE);
}

#if defined(NRF_UART_TYPE)
/**
 * @brief Configure pins for the UART/UARTE peripheral.
 *
 * @param uart UART/UARTE device.
 * @param pins Pins to be configured.
 * @param pin_cnt Number of pins.
 */
static void pinctrl_nrf_uart_config(NRF_UART_TYPE *uart,
				    const pinctrl_soc_pin_t *pins,
				    uint8_t pin_cnt)
{
	for (uint8_t i = 0U; i < pin_cnt; i++) {
		switch (NRF_GET_FUN(pins[i])) {
		case NRF_FUN_UART_TX:
			NRF_UART_PSEL(uart, TXD) = NRF_GET_PIN(pins[i]);
			nrf_gpio_pin_write(NRF_GET_PIN(pins[i]), 1);
			nrf_pin_configure(pins[i], NRF_GPIO_PIN_DIR_OUTPUT,
					  NRF_GPIO_PIN_INPUT_DISCONNECT);
			break;
		case NRF_FUN_UART_RX:
			NRF_UART_PSEL(uart, RXD) = NRF_GET_PIN(pins[i]);
			nrf_pin_configure(pins[i], NRF_GPIO_PIN_DIR_INPUT,
					  NRF_GPIO_PIN_INPUT_CONNECT);
			break;
		case NRF_FUN_UART_RTS:
			NRF_UART_PSEL(uart, RTS) = NRF_GET_PIN(pins[i]);
			nrf_gpio_pin_write(NRF_GET_PIN(pins[i]), 1);
			nrf_pin_configure(pins[i], NRF_GPIO_PIN_DIR_OUTPUT,
					  NRF_GPIO_PIN_INPUT_DISCONNECT);
			break;
		case NRF_FUN_UART_CTS:
			NRF_UART_PSEL(uart, CTS) = NRF_GET_PIN(pins[i]);
			nrf_pin_configure(pins[i], NRF_GPIO_PIN_DIR_INPUT,
					  NRF_GPIO_PIN_INPUT_CONNECT);
			break;
		default:
			break;
		}
	}
}
#endif /* defined(NRF_UART_TYPE) */

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			   uintptr_t reg)
{
	switch (reg) {
#if defined(NRF_UART_TYPE)
#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart0), okay)
	case DT_REG_ADDR(DT_NODELABEL(uart0)):
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart1), okay)
	case DT_REG_ADDR(DT_NODELABEL(uart1)):
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart2), okay)
	case DT_REG_ADDR(DT_NODELABEL(uart2)):
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart3), okay)
	case DT_REG_ADDR(DT_NODELABEL(uart3)):
#endif
		pinctrl_nrf_uart_config((NRF_UART_TYPE *)reg, pins, pin_cnt);
		break;
#endif /* defined(NRF_UART_TYPE) */
	default:
		return -ENOTSUP;
	}

	return 0;
}
