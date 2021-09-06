/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2021 Linaro Limited
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/clock_control/stm32_clock_control.h>
#include <drivers/pinctrl.h>
#include <gpio/gpio_stm32.h>
#include <pm/device_runtime.h>

#include <stm32_ll_bus.h>
#include <stm32_ll_gpio.h>

/**
 * @brief Array containing pointers to each GPIO port.
 *
 * Entries will be NULL if the GPIO port is not enabled.
 */
static const struct device * const gpio_ports[] = {
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpioa)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpiob)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpioc)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpiod)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpioe)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpiof)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpiog)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpioh)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpioi)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpioj)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpiok)),
};

/** Number of GPIO ports. */
static const size_t gpio_ports_cnt = ARRAY_SIZE(gpio_ports);

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32f1_pinctrl)
/**
 * @brief Helper function to check and apply provided pinctrl remap
 * configuration.
 *
 * Check operation verifies that pin remapping configuration is the same on all
 * pins. If configuration is valid AFIO clock is enabled and remap is applied
 *
 * @param pins List of pins to be configured.
 * @param pin_cnt Number of pins.
 * @param reg Device register address.
 *
 * @retval 0 If successful
 * @retval -EINVAL If pins have an incompatible set of remaps.
 */
static int stm32_pins_remap(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			    uintptr_t reg)
{
	uint8_t remap;

	remap = (uint8_t)STM32_DT_PINMUX_REMAP(pins[0].pinmux);

	/* check that all pins request the same remap */
	for (uint8_t i = 1U; i < pin_cnt; i++) {
		if (STM32_DT_PINMUX_REMAP(pins[i].pinmux) != remap) {
			return -EINVAL;
		}
	}

	/* A valid remapping configuration is available */
	/* Apply remapping before proceeding with pin configuration */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);

	switch (reg) {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can1), okay)
	case DT_REG_ADDR(DT_NODELABEL(can1)):
		if (remap == REMAP_1) {
			/* PB8/PB9 */
			LL_GPIO_AF_RemapPartial2_CAN1();
		} else if (remap == REMAP_2) {
			/* PD0/PD1 */
			LL_GPIO_AF_RemapPartial3_CAN1();
		} else {
			/* NO_REMAP: PA11/PA12 */
			LL_GPIO_AF_RemapPartial1_CAN1();
		}
		break;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can2), okay)
	case DT_REG_ADDR(DT_NODELABEL(can2)):
		if (remap == REMAP_1) {
			/* PB5/PB6 */
			LL_GPIO_AF_EnableRemap_CAN2();
		} else {
			/* PB12/PB13 */
			LL_GPIO_AF_DisableRemap_CAN2();
		}
		break;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1), okay)
	case DT_REG_ADDR(DT_NODELABEL(i2c1)):
		if (remap == REMAP_1) {
			LL_GPIO_AF_EnableRemap_I2C1();
		} else {
			LL_GPIO_AF_DisableRemap_I2C1();
		}
		break;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(timers1), okay)
	case DT_REG_ADDR(DT_NODELABEL(timers1)):
		if (remap == REMAP_1) {
			LL_GPIO_AF_RemapPartial_TIM1();
		} else if (remap == REMAP_2) {
			LL_GPIO_AF_EnableRemap_TIM1();
		} else {
			LL_GPIO_AF_DisableRemap_TIM1();
		}
		break;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(timers2), okay)
	case DT_REG_ADDR(DT_NODELABEL(timers2)):
		if (remap == REMAP_1) {
			LL_GPIO_AF_RemapPartial1_TIM2();
		} else if (remap == REMAP_2) {
			LL_GPIO_AF_RemapPartial2_TIM2();
		} else if (remap == REMAP_FULL) {
			LL_GPIO_AF_EnableRemap_TIM2();
		} else {
			LL_GPIO_AF_DisableRemap_TIM2();
		}
		break;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(timers3), okay)
	case DT_REG_ADDR(DT_NODELABEL(timers3)):
		if (remap == REMAP_1) {
			LL_GPIO_AF_RemapPartial_TIM3();
		} else if (remap == REMAP_2) {
			LL_GPIO_AF_EnableRemap_TIM3();
		} else {
			LL_GPIO_AF_DisableRemap_TIM3();
		}
		break;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(timers4), okay)
	case DT_REG_ADDR(DT_NODELABEL(timers4)):
		if (remap == REMAP_1) {
			LL_GPIO_AF_EnableRemap_TIM4();
		} else {
			LL_GPIO_AF_DisableRemap_TIM4();
		}
		break;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(timers9), okay)
	case DT_REG_ADDR(DT_NODELABEL(timers9)):
		if (remap == REMAP_1) {
			LL_GPIO_AF_EnableRemap_TIM9();
		} else {
			LL_GPIO_AF_DisableRemap_TIM9();
		}
		break;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(timers10), okay)
	case DT_REG_ADDR(DT_NODELABEL(timers10)):
		if (remap == REMAP_1) {
			LL_GPIO_AF_EnableRemap_TIM10();
		} else {
			LL_GPIO_AF_DisableRemap_TIM10();
		}
		break;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(timers11), okay)
	case DT_REG_ADDR(DT_NODELABEL(timers11)):
		if (remap == REMAP_1) {
			LL_GPIO_AF_EnableRemap_TIM11();
		} else {
			LL_GPIO_AF_DisableRemap_TIM11();
		}
		break;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(timers12), okay)
	case DT_REG_ADDR(DT_NODELABEL(timers12)):
		if (remap == REMAP_1) {
			LL_GPIO_AF_EnableRemap_TIM12();
		} else {
			LL_GPIO_AF_DisableRemap_TIM12();
		}
		break;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(timers13), okay)
	case DT_REG_ADDR(DT_NODELABEL(timers13)):
		if (remap == REMAP_1) {
			LL_GPIO_AF_EnableRemap_TIM13();
		} else {
			LL_GPIO_AF_DisableRemap_TIM13();
		}
		break;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(timers14), okay)
	case DT_REG_ADDR(DT_NODELABEL(timers14)):
		if (remap == REMAP_1) {
			LL_GPIO_AF_EnableRemap_TIM14();
		} else {
			LL_GPIO_AF_DisableRemap_TIM14();
		}
		break;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(timers15), okay)
	case DT_REG_ADDR(DT_NODELABEL(timers15)):
		if (remap == REMAP_1) {
			LL_GPIO_AF_EnableRemap_TIM15();
		} else {
			LL_GPIO_AF_DisableRemap_TIM15();
		}
		break;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(timers16), okay)
	case DT_REG_ADDR(DT_NODELABEL(timers16)):
		if (remap == REMAP_1) {
			LL_GPIO_AF_EnableRemap_TIM16();
		} else {
			LL_GPIO_AF_DisableRemap_TIM16();
		}
		break;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(timers17), okay)
	case DT_REG_ADDR(DT_NODELABEL(timers17)):
		if (remap == REMAP_1) {
			LL_GPIO_AF_EnableRemap_TIM17();
		} else {
			LL_GPIO_AF_DisableRemap_TIM17();
		}
		break;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(usart1), okay)
	case DT_REG_ADDR(DT_NODELABEL(usart1)):
		if (remap == REMAP_1) {
			LL_GPIO_AF_EnableRemap_USART1();
		} else {
			LL_GPIO_AF_DisableRemap_USART1();
		}
		break;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(usart2), okay)
	case DT_REG_ADDR(DT_NODELABEL(usart2)):
		if (remap == REMAP_1) {
			LL_GPIO_AF_EnableRemap_USART2();
		} else {
			LL_GPIO_AF_DisableRemap_USART2();
		}
		break;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(usart3), okay)
	case DT_REG_ADDR(DT_NODELABEL(usart3)):
		if (remap == REMAP_2) {
			LL_GPIO_AF_EnableRemap_USART3();
		} else if (remap == REMAP_1) {
			LL_GPIO_AF_RemapPartial_USART3();
		} else {
			LL_GPIO_AF_DisableRemap_USART3();
		}
		break;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi1), okay)
	case DT_REG_ADDR(DT_NODELABEL(spi1)):
		if (remap == REMAP_1) {
			LL_GPIO_AF_EnableRemap_SPI1();
		} else {
			LL_GPIO_AF_DisableRemap_SPI1();
		}
		break;
#endif
	}

	return 0;
}
#endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32f1_pinctrl) */

static int stm32_pin_configure(uint32_t pin, uint32_t func, uint32_t altf)
{
	const struct device *port_device;
	int ret = 0;

	if (STM32_PORT(pin) >= gpio_ports_cnt) {
		return -EINVAL;
	}

	port_device = gpio_ports[STM32_PORT(pin)];

	if ((port_device == NULL) || (!device_is_ready(port_device))) {
		return -ENODEV;
	}

#ifdef CONFIG_PM_DEVICE_RUNTIME
	ret = pm_device_get(port_device);
	if (ret < 0) {
		return ret;
	}
#endif

	gpio_stm32_configure(port_device, STM32_PIN(pin), func, altf);

#ifdef CONFIG_PM_DEVICE_RUNTIME
	ret = pm_device_put(port_device);
#endif

	return ret;
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			   uintptr_t reg)
{
	uint32_t pin, mux;
	uint32_t func = 0;
	int ret = 0;

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32f1_pinctrl)
	ret = stm32_pins_remap(pins, pin_cnt, reg);
	if (ret < 0) {
		return ret;
	}
#else
	ARG_UNUSED(reg);
#endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32f1_pinctrl) */

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		mux = pins[i].pinmux;

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32f1_pinctrl)
		uint32_t pupd;

		if (STM32_DT_PINMUX_FUNC(mux) == ALTERNATE) {
			func = pins[i].pincfg | STM32_MODE_OUTPUT | STM32_CNF_ALT_FUNC;
		} else if (STM32_DT_PINMUX_FUNC(mux) == ANALOG) {
			func = pins[i].pincfg | STM32_MODE_INPUT | STM32_CNF_IN_ANALOG;
		} else if (STM32_DT_PINMUX_FUNC(mux) == GPIO_IN) {
			func = pins[i].pincfg | STM32_MODE_INPUT;
			pupd = func & (STM32_PUPD_MASK << STM32_PUPD_SHIFT);
			if (pupd == STM32_PUPD_NO_PULL) {
				func = func | STM32_CNF_IN_FLOAT;
			} else {
				func = func | STM32_CNF_IN_PUPD;
			}
		} else {
			/* Not supported */
			__ASSERT_NO_MSG(STM32_DT_PINMUX_FUNC(mux));
		}
#else
		if (STM32_DT_PINMUX_FUNC(mux) < STM32_ANALOG) {
			func = pins[i].pincfg | STM32_MODER_ALT_MODE;
		} else if (STM32_DT_PINMUX_FUNC(mux) == STM32_ANALOG) {
			func = STM32_MODER_ANALOG_MODE;
		} else {
			/* Not supported */
			__ASSERT_NO_MSG(STM32_DT_PINMUX_FUNC(mux));
		}
#endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32f1_pinctrl) */

		pin = STM32PIN(STM32_DT_PINMUX_PORT(mux),
			       STM32_DT_PINMUX_LINE(mux));

		ret = stm32_pin_configure(pin, func, STM32_DT_PINMUX_FUNC(mux));
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}
