/*
 * owi_stm32f030.h
 *
 *  Created on: 28 ���� 2015 �.
 *      Author: kulish_y
 */

#ifndef OWI_STM32F030_H_
#define OWI_STM32F030_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <libopencm3/stm32/gpio.h>
#include "owi.h"



typedef struct
{
	uint32_t gpio;
	uint16_t pins;
} OWI_STM32F0xx_Pin;

typedef struct
{
	OWI_GPIO_Interface interface;
	OWI_STM32F0xx_Pin pin;
} OWI_STM32F0xx_GPIO_Driver;

extern const OWI_GPIO_Interface OWI_STM32F0XX_PINDRIVER_INTERFACE;

#ifdef __cplusplus
}
#endif


#endif /* OWI_STM32F030_H_ */
