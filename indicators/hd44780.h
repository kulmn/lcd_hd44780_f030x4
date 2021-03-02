#ifndef HC44780_H_
#define HC44780_H_

#include <stdint.h>


typedef enum { HD44780_RESULT_OK, HD44780_RESULT_ERROR } HD44780_Result;
typedef enum { HD44780_PINSTATE_LOW, HD44780_PINSTATE_HIGH } HD44780_PinState;
typedef enum { HD44780_MODE_4BIT, HD44780_MODE_8BIT } HD44780_Mode;
typedef enum { HD44780_CHARSIZE_5x8, HD44780_CHARSIZE_5x10 } HD44780_CharSize;


struct HD44780_Driver;
typedef struct HD44780_Driver HD44780_Interface;

/* Hardware abstraction layer */
struct HD44780_Driver
{
	HD44780_Result (*hd44780_driver_init)(HD44780_Interface *interface, HD44780_Mode addr_mode);
	HD44780_Result (*hd44780_driver_send)(HD44780_Interface *interface, uint8_t value, HD44780_PinState rs_mode);
};

typedef void (*HD44780_AssertFn)(const char *filename, unsigned long line);
typedef void (*HD44780_DelayMicrosecondsFn)(uint16_t us);


/* HD44780 control structure */
typedef struct
{
  HD44780_Interface *driver;
  HD44780_DelayMicrosecondsFn delay_microseconds;
  uint8_t displayfunction;
  uint8_t displaycontrol;
  uint8_t displaymode;
} HD44780;


HD44780_Result hd44780_init(HD44780 *display, HD44780_Mode addr_mode, uint8_t columns, uint8_t rows, HD44780_CharSize charsize);
HD44780_Result hd44780_write_char(HD44780 *display, char c);
HD44780_Result hd44780_write_string(HD44780 *display, const char *s);
HD44780_Result hd44780_clear(HD44780 *display);
HD44780_Result hd44780_home(HD44780 *display);
HD44780_Result hd44780_scroll_left(HD44780 *display);
HD44780_Result hd44780_scroll_right(HD44780 *display);
HD44780_Result hd44780_left_to_right(HD44780 *display);
HD44780_Result hd44780_right_to_left(HD44780 *display);
HD44780_Result hd44780_create_char(HD44780 *display, uint8_t code, const uint8_t *charmap);
HD44780_Result hd44780_move_cursor(HD44780 *display, uint8_t column, uint8_t row);
HD44780_Result hd44780_display_on(HD44780 *display);
HD44780_Result hd44780_display_off(HD44780 *display);
HD44780_Result hd44780_blink_on(HD44780 *display);
HD44780_Result hd44780_blink_off(HD44780 *display);
HD44780_Result hd44780_cursor_on(HD44780 *display);
HD44780_Result hd44780_cursor_off(HD44780 *display);
HD44780_Result hd44780_autoscroll_on(HD44780 *display);
HD44780_Result hd44780_autoscroll_off(HD44780 *display);
HD44780_Result hd44780_backlight_on(HD44780 *display);
HD44780_Result hd44780_backlight_off(HD44780 *display);

#define HD44780_MAKE_5BITS(b4,b3,b2,b1,b0) \
    (((b0) & 1) | \
     ((b1) & 1) << 1 | \
     ((b2) & 1) << 2 | \
     ((b3) & 1) << 3 | \
     ((b4) & 1) << 4)


#endif /* HC44780_H_ */
