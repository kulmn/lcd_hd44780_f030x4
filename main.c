/*
 * main.c
 *
 *  Created on: 08 сент. 2014 г.
 *      Author: yura
 */

#include "main.h"



//xQueueHandle xTemperQueue;

SemaphoreHandle_t xButtons_Smphr[6];

UBaseType_t uxHighWaterMark1;
UBaseType_t uxHighWaterMark;

HD44780 lcd;
HD44780_Driver lcd_driver;

SLIP slip_pr;
SLIP_USART_Driver slip_pr_driver;

//OWI t_sensor;
//OWI_STM32F0xx_GPIO_Driver t_sensor_pindriver;


USART_HAL usart_1;

#define USART1_RX_BUF_SIZE	128
#define USART1_TX_BUF_SIZE	32

static uint8_t usart1_rx_buf[USART1_RX_BUF_SIZE];
static uint8_t usart1_tx_buf[USART1_TX_BUF_SIZE];




const Pin button_pins[6] = { {BUTTON_0}, {BUTTON_1}, {BUTTON_2}, {BUTTON_3}, {BUTTON_4}, {BUTTON_5} };
uint8_t bright=80, contr=30;
uint8_t lcd_buf_line_0[17];
uint8_t lcd_buf_line_1[17];

void slim_protocol_init(void)
{
	slip_pr_driver.usart_drv = &usart_1;
	slip_pr_driver.interface = SLIP_USART_INTERFACE;

	slip_pr.bytes_received = 0;
	slip_pr.max_packet_size = 32;
	slip_pr.driver = (SLIP_Interface*) &slip_pr_driver;

}


void init_lcd(void)
{

  lcd_driver.pin_mask[HD44780_PIN_RS] = 1 << 1;
  lcd_driver.pin_mask[HD44780_PIN_EN] = 1 << 2;
  lcd_driver.pin_mask[HD44780_PIN_RW] = 0;
  lcd_driver.pin_mask[HD44780_PIN_DP0] = 1 << 9;
  lcd_driver.pin_mask[HD44780_PIN_DP1] = 1 << 10;
  lcd_driver.pin_mask[HD44780_PIN_DP2] = 1 << 11;
  lcd_driver.pin_mask[HD44780_PIN_DP3] = 1 << 12;
  lcd_driver.pin_mask[HD44780_PIN_DP4] = 1 << 3;
  lcd_driver.pin_mask[HD44780_PIN_DP5] = 1 << 4;
  lcd_driver.pin_mask[HD44780_PIN_DP6] = 1 << 5;
  lcd_driver.pin_mask[HD44780_PIN_DP7] = 1 << 6;

  lcd_driver.interface = HD44780_INTERFACE;
  lcd_driver.delay_us = delay_us;

  lcd.delay_microseconds = delay_us;
  lcd.driver = (HD44780_Interface*)&lcd_driver;

  // инициализируем дисплей: 16x2, 4-битный интерфейс, символы 5x8 точек.
  hd44780_init(&lcd, HD44780_MODE_4BIT, 16, 2, HD44780_CHARSIZE_5x8);
}


void vLcdOutTask (void *pvParameters)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 100;
//	portBASE_TYPE xStatus;

	xLastWakeTime = xTaskGetTickCount();

	for( ;; )
	{
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

    	hd44780_move_cursor(&lcd, 0, 0);
    	hd44780_write_string(&lcd,lcd_buf_line_0);

    	hd44780_move_cursor(&lcd, 0, 1);
    	hd44780_write_string(&lcd,lcd_buf_line_1);

	}
    vTaskDelete(NULL);
}


void vReceiveDataTask (void *pvParameters)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 30;
	uint8_t		buf[32];
	uint8_t		size=0;

	xLastWakeTime = xTaskGetTickCount();

	for( ;; )
    {
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

		size = slip_recv_packet(&slip_pr, buf);

		if (size)
		{
			if ( ComputeCRC16(buf, size) == 0 && (buf[0] == DEVICE_ADDR) )
			{
				switch(buf[1])
				{
						case LCD_LINE0:
						{
							memcpy(lcd_buf_line_0, buf+3, buf[2]);
							hd44780_move_cursor(&lcd, 0, 0);
							hd44780_write_string(&lcd,lcd_buf_line_0);
							break;
						}
						case LCD_LINE1:
						{
							memcpy(lcd_buf_line_1, buf+3, buf[2]);
							hd44780_move_cursor(&lcd, 0, 1);
							hd44780_write_string(&lcd,lcd_buf_line_1);
							break;
						}

						case BRIGHT_CONTR:
						{
							bright = buf[3];
							contr = buf[4];
							HD44780_Set_Br_Cntr(bright, contr);
							break;
						}
				}
			}
		}

    }
	vTaskDelete(NULL);
}


void vGreenLedTask (void *pvParameters)
{
    while(1)
    {
//    	gpio_toggle(LED_GREEN);	/* LED on/off */
//    	vTaskDelay(500);
//    	gpio_clear(LED_GREEN);
//    	vTaskDelay(500);

     }
    vTaskDelete(NULL);
}


void vSendButtonDataTask (void *pvParameters)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 10;
	uint8_t		buf[16];
	uint8_t		flag=0;

	xLastWakeTime = xTaskGetTickCount();

	for( ;; )
    {
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

    	for (uint8_t i=0; i<6; i++)
    	{
    		buf[i+3] = uxSemaphoreGetCount(xButtons_Smphr[i]);
    		if( (buf[i+3]) != 0 )
    	    {
    			flag=1;
    	    	while(xSemaphoreTake( xButtons_Smphr[i], ( portTickType ) 0 ) == pdTRUE) {}
    		} else	buf[i+3]=0;
    	}

    	if (flag)
    	{
    		flag=0;
    		buf[0]=DEVICE_ADDR;
    		buf[1]=BUTTONS_DAT;
    		buf[2]=6;

    		uint16_t crc = ComputeCRC16(buf, 9);

    		buf[9] = crc & 0x00FF;
    		buf[10] = (crc & 0xFF00) >> 8;

    		slip_send_packet(&slip_pr,buf, 11);
    	}

	}
    vTaskDelete(NULL);
}


/******************************************************************************/
void vGetButtonStateTask (void *pvParameters)			// ~ ???  bytes of stack used
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 10;						// 1000/10ms = 100 Hz
	uint8_t buttons_cnt[] = {0,0,0,0,0,0};

	xLastWakeTime = xTaskGetTickCount();

	for (uint8_t i=0; i<6; i++) xButtons_Smphr[i] = xSemaphoreCreateCounting(10,0);

	for( ;; )
	{
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

		for (uint8_t i=0; i<6; i++)
		{
			if(!gpio_get(button_pins[i].gpio, button_pins[i].pin))
			{
				buttons_cnt[i]++;
				if( buttons_cnt[i] > 100) {buttons_cnt[i] = 80; xSemaphoreGive( xButtons_Smphr[i] ); }
				if( buttons_cnt[i] == 2) { xSemaphoreGive( xButtons_Smphr[i] ); }
			} else buttons_cnt[i] = 0;
		}

	}
	vTaskDelete(NULL);
}



/******************************************************************************/
static void system_clock_setup(void)
{
	// Enable external high-speed oscillator 4MHz.
//	rcc_osc_on(RCC_HSE);
//	rcc_wait_for_osc_ready(RCC_HSE);
	rcc_set_sysclk_source(RCC_HSI);

	rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
	rcc_set_ppre(RCC_CFGR_PPRE_NODIV);

	flash_set_ws(FLASH_ACR_LATENCY_000_024MHZ);

	// 4MHz * 4  = 16MHz
	rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_MUL4);

	RCC_CFGR &= ~RCC_CFGR_PLLSRC;

	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);
	rcc_set_sysclk_source(RCC_PLL);

	rcc_apb1_frequency = 16000000;
	rcc_ahb_frequency = 16000000;
}

/******************************************************************************/
void usart_setup(void)
{
	usart_1.usart=USART1;
	usart_1.baudrate=57600;
	usart_1.rx_buf_ptr=(uint8_t *)usart1_rx_buf;
	usart_1.tx_buf_ptr=(uint8_t *)usart1_tx_buf;
	usart_1.rx_buf_size=USART1_RX_BUF_SIZE;
	usart_1.tx_buf_size=USART1_TX_BUF_SIZE;

	// Setup GPIO pins for usart1 transmit.
	gpio_mode_setup(PORT(USART1_TX_PIN), GPIO_MODE_AF, GPIO_PUPD_NONE, PIN(USART1_TX_PIN));
	// Setup usart1 TX pin as alternate function.
	gpio_set_af(PORT(USART1_TX_PIN), GPIO_AF1, PIN(USART1_TX_PIN));

	// Setup GPIO pins for usart1 receive.
	gpio_mode_setup(PORT(USART1_RX_PIN), GPIO_MODE_AF, GPIO_PUPD_NONE, PIN(USART1_RX_PIN));
	// Setup usart1 RX pin as alternate function.
	gpio_set_af(PORT(USART1_RX_PIN), GPIO_AF1, PIN(USART1_RX_PIN));

	rcc_periph_clock_enable(RCC_USART1);
	uart_init(&usart_1);
	// Enable the usart1 interrupt.
	nvic_enable_irq(NVIC_USART1_IRQ);
}



/******************************************************************************/
void HD44780_Set_Br_Cntr(uint8_t br, uint8_t contr)
{
	pwm_set_pulse_width(TIM1, TIM_OC2, br);
	pwm_set_pulse_width(TIM1, TIM_OC3, contr);
}


/******************************************************************************/
void HD44780_Br_Cntr_pwm_init(void)
{
	pwm_init_timer( TIM1,RCC_TIM1, (rcc_apb1_frequency/1000000-1), 100);		// 16 KHz  Period=100 16/100=1.6 KHz
	pwm_init_output_channel(TIM1, TIM_OC2, PORT(TIM1_PWM_CH2), PIN(TIM1_PWM_CH2), GPIO_AF2);
	pwm_set_pulse_width(TIM1, TIM_OC2, 80);

	pwm_init_output_channel(TIM1, TIM_OC3, PORT(TIM1_PWM_CH3), PIN(TIM1_PWM_CH3), GPIO_AF2);
	pwm_set_pulse_width(TIM1, TIM_OC3, 30);

     pwm_start_timer(TIM1);
}

/******************************************************************************/
//----------------------------
// initializations
//----------------------------
void vFreeRTOSInitAll()
{
	system_clock_setup();
	rcc_peripheral_enable_clock(&RCC_APB1ENR ,RCC_APB1ENR_PWREN);				// enable APB1 clocks

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOF);

//	gpio_mode_setup(PORT(LED_GREEN), GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN(LED_GREEN));
//	gpio_mode_setup(PORT(SPI1_CS), GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN(SPI1_CS));
//	gpio_mode_setup(PORT(TIM1_PWM_CH3), GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN(TIM1_PWM_CH3));


	// Buttons init
	for (uint8_t i=0; i<6; i++)
	{
		gpio_mode_setup(button_pins[i].gpio, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, button_pins[i].pin);
	}

	delay_init();
	HD44780_Br_Cntr_pwm_init();

	HC595_spi_init();

//	init_owi();
//	DS18B20_Init(&t_sensor, DS18B20_12BIT_RES);

//	RTC_Set_Time(CLOCK_INIT);		// 12:30
	init_lcd();

	usart_setup();

	slim_protocol_init();

}



void vApplicationIdleHook( void )
{
//	gpio_clear(LED_GREEN);
//	GPIO_SET(LED_GREEN);
//	GPIO_CLR(LED_YELLOW);

}

void vApplicationTickHook( void )
{
//	gpio_set(LED_GREEN);
//	GPIO_CLR(LED_GREEN);
//	GPIO_SET(LED_YELLOW);

}

/******************************************************************************/
int main(void)
{
	vFreeRTOSInitAll();
//	xTemperQueue=xQueueCreate( 5, sizeof( DS18B20_TypeDef ));
//	xTaskCreate(vGetTempTask,(signed char*)"", configMINIMAL_STACK_SIZE * 4 ,&t_sensor, tskIDLE_PRIORITY + 1, NULL);
//	xTaskCreate(vGreenLedTask,(signed char*)"", configMINIMAL_STACK_SIZE,	NULL, tskIDLE_PRIORITY + 1, NULL);
//	xTaskCreate(vLcdOutTask,(signed char*)"", configMINIMAL_STACK_SIZE * 4,	NULL, tskIDLE_PRIORITY + 1, NULL);


	xTaskCreate(vSendButtonDataTask,(signed char*)"", configMINIMAL_STACK_SIZE * 2,	NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(vGetButtonStateTask,(signed char*)"", configMINIMAL_STACK_SIZE * 4,	NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(vReceiveDataTask,(signed char*)"", configMINIMAL_STACK_SIZE * 2,	NULL, tskIDLE_PRIORITY + 1, NULL);
	vTaskStartScheduler();


	for( ;; );
}
