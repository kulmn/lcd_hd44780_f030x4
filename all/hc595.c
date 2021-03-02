/*
 * hc595.c
 *
 *  Created on: 3 ����. 2016 �.
 *      Author: kulish_y
 */

#include "hc595.h"

/******************************************************************************/
void HC595_spi_init(void)
{
	gpio_mode_setup(PORT(SPI1_SCK), GPIO_MODE_AF, GPIO_PUPD_NONE, PIN(SPI1_SCK)); // SCK
	gpio_mode_setup(PORT(SPI1_MOSI), GPIO_MODE_AF, GPIO_PUPD_NONE, PIN(SPI1_MOSI));	// MISO

	gpio_mode_setup(PORT(SPI1_CS), GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN(SPI1_CS));	// CS
	gpio_set(SPI1_CS);

	// Enable SPI1 Periph and gpio clocks
	rcc_periph_clock_enable(RCC_SPI1);

//	spi_reset(SPI1);
	spi_set_master_mode(SPI1);

	spi_set_bidirectional_transmit_only_mode(SPI1);

//	spi_set_unidirectional_mode(SPI1);
//	spi_set_receive_only_mode(SPI1);
	spi_set_data_size(SPI1,SPI_CR2_DS_16BIT);
	spi_set_clock_polarity_0(SPI1);
	spi_set_clock_phase_0(SPI1);
	spi_enable_software_slave_management(SPI1);
	spi_set_nss_high(SPI1);
	spi_set_baudrate_prescaler(SPI1,SPI_CR1_BR_FPCLK_DIV_8);
	spi_send_msb_first(SPI1);
	spi_disable_crc(SPI1);

	spi_enable(SPI1);

}

/******************************************************************************/
void HC595_send(uint16_t data)
{
	gpio_clear(SPI1_CS);
	spi_send(SPI1,data);
//	spi_send8(SPI1,data);
	while ((SPI_SR(SPI1) & SPI_SR_BSY));
	gpio_set(SPI1_CS);
}

/******************************************************************************
void HC595_send(uint16_t data)
{
	gpio_clear(SPI1_CS);
	spi_send(SPI1,data);
//	spi_send8(SPI1,data);
	while ((SPI_SR(SPI1) & SPI_SR_BSY));
	gpio_set(SPI1_CS);
}
*/

