#ifndef HD44780_DRIVER_SR_H_
#define HD44780_DRIVER_SR_H_


#include <stdlib.h>
#include <libopencm3/stm32/gpio.h>
#include "hd44780.h"
#include "hc595.h"


/* Abstract HD44780 pin definition */
typedef enum
{
  HD44780_PIN_RS,        // LOW: command.  HIGH: character.
  HD44780_PIN_EN,    // latch, activated by a HIGH pulse.
  HD44780_PIN_RW,        // optional; LOW: write to LCD, HIGH: read from LCD
  /* Data pins DP0..DP7; in 4-bit mode DP0..DP3 are not needed. */
  HD44780_PIN_DP0,
  HD44780_PIN_DP1,
  HD44780_PIN_DP2,
  HD44780_PIN_DP3,
  HD44780_PIN_DP4,
  HD44780_PIN_DP5,
  HD44780_PIN_DP6,
  HD44780_PIN_DP7,
  HD44780_PINS_AMOUNT // enum member counter, must be last
} HD44780_Pin;



typedef struct
{
  HD44780_Interface interface;
  HD44780_Mode 		addr_mode;
  uint16_t 			out_buf;
  HD44780_Pin 		dp_first;
  uint8_t 			dp_amount;
  HD44780_DelayMicrosecondsFn delay_us;
  uint16_t pin_mask[HD44780_PINS_AMOUNT];
} HD44780_Driver;

extern const HD44780_Interface HD44780_INTERFACE;


#endif /* HD44780_DRIVER_SR_H_ */
