/*
 * LT_HAL_SPI.h
 *
 *  Created on: Jul 19, 2025
 *      Author: awnin
 */

/*! @file
 @ingroup LT_HAL_SPI
 Library Header File for LT_HAL_SPI: Routines to communicate with STM32F091RCT6's hardware SPI port.
 */

#ifndef INC_LT_HAL_SPI_H_
#define INC_LT_HAL_SPI_H_

#include <stdint.h>

void cs_low(uint8_t pin);

void cs_high(uint8_t pin);

void delay_u(uint16_t micro);

void delay_m(uint16_t milli);

void spi_enable();

/*
 Writes an array of bytes out of the SPI port
 */
void spi_write_array(uint8_t len, // Option: Number of bytes to be written on the SPI port
		uint8_t data[] //Array of bytes to be written on the SPI port
		);

/*
 Writes and read a set number of bytes using the SPI port.

 */

void spi_write_read(uint8_t tx_Data[], //array of data to be written on SPI port
		uint8_t tx_len, //length of the tx data arry
		uint8_t *rx_data, //Input: array that will store the data read by the SPI port
		uint8_t rx_len //Option: number of bytes to be read from the SPI port
		);

uint8_t spi_read_byte(uint8_t tx_dat);

#endif /* INC_LT_HAL_SPI_H_ */
