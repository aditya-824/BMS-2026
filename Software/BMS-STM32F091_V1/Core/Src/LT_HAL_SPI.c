/*
 * LT_HAL_SPI.c
 *
 *  Created on: Jul 19, 2025
 *      Author: awnin
 */

#include <stdint.h>
#include <stdlib.h>
#include "stm32f0xx_hal.h"
#include "LT_HAL_SPI.h"

#define output_low(pin)   HAL_GPIO_WritePin(CS_GPIO_PORT, pin, RESET)
#define output_high(pin)  HAL_GPIO_WritePin(CS_GPIO_PORT, pin, SET)
#define CS_GPIO_PORT GPIOA // GPIO Port for software CS pin

extern SPI_HandleTypeDef hspi1; // SPI Handle
extern TIM_HandleTypeDef htim2; // Timer Handle

void cs_low(uint8_t pin)
{
  output_low(pin);
}

void cs_high(uint8_t pin)
{
  output_high(pin);
}

void delay_u(uint16_t micro)
{
	__HAL_TIM_SET_COUNTER(&htim2,0);  // set the counter value a 0
		while (__HAL_TIM_GET_COUNTER(&htim2) < micro);  // wait for the counter to reach the us input
}

void delay_m(uint16_t milli)
{
  HAL_Delay(milli);
}

// Setup the processor for hardware SPI communication.
// Must be called before using the other SPI routines.
// Alternatively, call quikeval_SPI_connect(), which automatically
// calls this function.
void spi_enable() // Configures SCK frequency. Use constant defined in header file.
{
  //pinMode(SCK, OUTPUT);             //! 1) Setup SCK as output
  //pinMode(MOSI, OUTPUT);            //! 2) Setup MOSI as output
  //pinMode(QUIKEVAL_CS, OUTPUT);     //! 3) Setup CS as output
//  SPI.begin();
//  SPI.setClockDivider(spi_clock_divider);
}

/*
Writes an array of bytes out of the SPI port
*/
void spi_write_array(uint8_t len, // Option: Number of bytes to be written on the SPI port
                     uint8_t data[] //Array of bytes to be written on the SPI port
                    )
{
	uint8_t i;
	uint8_t rx_data;

  for (i = 0; i < len; i++)
  {
//	  SPI.transfer((int8_t)data[i]);
	  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&data[i], &rx_data, 1, HAL_MAX_DELAY);
  }
}

/*
 Writes and read a set number of bytes using the SPI port.

*/

void spi_write_read(uint8_t tx_Data[],//array of data to be written on SPI port
                    uint8_t tx_len, //length of the tx data arry
                    uint8_t *rx_data,//Input: array that will store the data read by the SPI port
                    uint8_t rx_len //Option: number of bytes to be read from the SPI port
                   )
{
	uint8_t i;
	uint8_t data;

  for (i = 0; i < tx_len; i++)
  {
//	  SPI.transfer(tx_Data[i]);
	  HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&tx_Data[i], &data, 1, HAL_MAX_DELAY);
  }

  for (uint8_t i = 0; i < rx_len; i++)
  {
//    rx_data[i] = (uint8_t)SPI.transfer(0xFF);
	  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)0xFF, (uint8_t*)&rx_data[i], 1, HAL_MAX_DELAY);
  }

}


uint8_t spi_read_byte(uint8_t tx_dat)
{
  uint8_t data;
//  data = (uint8_t)SPI.transfer(0xFF);
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)0xFF, (uint8_t *)&data, 1, HAL_MAX_DELAY);
  return(data);
}
