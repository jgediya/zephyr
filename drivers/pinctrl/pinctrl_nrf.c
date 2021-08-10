/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/pinctrl.h>

#include <hal/nrf_gpio.h>

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

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			   uintptr_t reg)
{
	switch (reg) {
	default:
		return -ENOTSUP;
	}

	return 0;
}
