/*
 * hc595.h
 *
 *  Created on: 3 но€б. 2016 г.
 *      Author: kulish_y
 */

#ifndef ALL_INCLUDE_HC595_H_
#define ALL_INCLUDE_HC595_H_

#include <stdint.h>
#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>



#define _PORT(Prt,Pn)         (Prt)
#define _PIN(Prt,Pn)         (Pn)

#define PORT(PP)        _PORT(PP)
#define PIN(PP)         _PIN(PP)

#define SPI1_SCK			GPIOA, GPIO5
#define SPI1_MOSI			GPIOA, GPIO7
#define SPI1_CS				GPIOA, GPIO6


void HC595_spi_init(void);
void HC595_send(uint16_t data);

#endif /* ALL_INCLUDE_HC595_H_ */
