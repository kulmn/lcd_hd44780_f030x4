/*
 * main.h
 *
 *  Created on: 21 ���. 2014 �.
 *      Author: kulish_y
 */

#ifndef MAIN_H_
#define MAIN_H_


#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/spi.h>


// FreeRTOS inc
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


#include "delay.h"
#include "pwm.h"
//#include "pid.h"
#include "convert_fn.h"
#include <usart_hl.h>
#include "hc595.h"

//#include "owi.h"
//#include "owi_stm32f0xx.h"
#include "hd44780.h"
#include "hd44780_driver_sr.h"
//#include "ds18b20.h"
#include <slip_prot.h>
#include <slip_prot_uart_drv.h>


//#define CRITICAL_SECTION_START	taskENTER_CRITICAL();
//#define CRITICAL_SECTION_END	taskEXIT_CRITICAL();



#define _PORT(Prt,Pn)         (Prt)
#define _PIN(Prt,Pn)         (Pn)

#define PORT(PP)        _PORT(PP)
#define PIN(PP)         _PIN(PP)



/**** PINs defines *******/

#define USART1_TX_PIN	GPIOA, GPIO2
#define USART1_RX_PIN	GPIOA, GPIO3

//#define SPI1_SCK			GPIOA, GPIO5
//#define SPI1_MOSI			GPIOA, GPIO7
//#define SPI1_CS				GPIOA, GPIO6

//#define SPI1_MISO			GPIOA, GPIO6
//#define SPI1_CS					GPIOA, GPIO0


#define TIM1_PWM_CH2			GPIOA, GPIO9
#define TIM1_PWM_CH3			GPIOA, GPIO10



// Buttons def
#define BUTTON_0		GPIOF, GPIO0
#define BUTTON_1		GPIOF, GPIO1
#define BUTTON_2		GPIOA, GPIO0
#define BUTTON_3		GPIOA, GPIO1
#define BUTTON_4		GPIOA, GPIO4
#define BUTTON_5		GPIOB, GPIO1


#define DEVICE_ADDR		0x01

// functions
#define BUTTONS_DAT			0x05
#define LCD_LINE0		0x0A
#define LCD_LINE1		0x0B
#define BRIGHT_CONTR		0x10


typedef struct
{
  uint32_t gpio;
  uint16_t pin;
} Pin;



//void RTC_Set_Time(uint8_t,uint8_t);
//void Correct_Rtc_Time(void);
void periphery_init(void);
void HD44780_Set_Br_Cntr(uint8_t br, uint8_t contr);
void sleep_mode(void);



#endif /* MAIN_H_ */
