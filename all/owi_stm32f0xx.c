/*
 * owi_stm32f030.c
 *
 *  Created on: 28 ���� 2015 �.
 *      Author: kulish_y
 */

#include "owi_stm32f0xx.h"


static OWI_Result owi_pin_configure(OWI_GPIO_Interface *interface, OWI_PinMode mode)
{
	OWI_STM32F0xx_GPIO_Driver *driver = (OWI_STM32F0xx_GPIO_Driver*)interface;
	OWI_STM32F0xx_Pin *hw_pin = &driver->pin;

//	hw_pin->gpio
//	driver->pin->gpio;


	return OWI_RESULT_OK;
}

static OWI_Result owi_pull_bus_low( OWI_GPIO_Interface *interface)
{
	OWI_STM32F0xx_GPIO_Driver *driver = (OWI_STM32F0xx_GPIO_Driver*)interface;
	OWI_STM32F0xx_Pin *hw_pin = &driver->pin;

	gpio_mode_setup(hw_pin->gpio, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, hw_pin->pins);
	gpio_clear(hw_pin->gpio, hw_pin->pins);
	//gpio_set_output_options(hw_pin->gpio, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, hw_pin->pins);

	//_GPIO_MODER(hw_pin->gpio, hw_pin->pins, MODE_OUTPUT);
	//_GPIO_CLR(hw_pin->gpio, hw_pin->pins);

	return OWI_RESULT_OK;
}

static OWI_Result owi_release_bus(OWI_GPIO_Interface *interface)
{
	OWI_STM32F0xx_GPIO_Driver *driver = (OWI_STM32F0xx_GPIO_Driver*)interface;
	OWI_STM32F0xx_Pin *hw_pin = &driver->pin;

	gpio_mode_setup(hw_pin->gpio, GPIO_MODE_INPUT, GPIO_PUPD_NONE, hw_pin->pins);

	//_GPIO_MODER(hw_pin->gpio, hw_pin->pins, MODE_INPUT);

	return OWI_RESULT_OK;
}

static OWI_PinState owi_get_bus_state(OWI_GPIO_Interface *interface)
{
	OWI_STM32F0xx_GPIO_Driver *driver = (OWI_STM32F0xx_GPIO_Driver*)interface;
	OWI_STM32F0xx_Pin *hw_pin = &driver->pin;

	if (gpio_get(hw_pin->gpio, hw_pin->pins)) return OWI_PINSTATE_HIGH;
	else return OWI_PINSTATE_LOW;
}



const OWI_GPIO_Interface OWI_STM32F0XX_PINDRIVER_INTERFACE =
{
	owi_pin_configure,
	owi_pull_bus_low,
	owi_release_bus,
	owi_get_bus_state
};

