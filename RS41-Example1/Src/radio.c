/*
 * radio.c
 *
 *  Created on: Dec 16, 2017
 *      Author: Milosz Iskrzynski SQ6NTI
 */

#include "radio.h"

SPI_HandleTypeDef *radioSpi;
GPIO_TypeDef *nselGpio;
uint16_t nselPin;

void radioInit(SPI_HandleTypeDef *spi, GPIO_TypeDef *gpio, uint16_t pin) {
	/* Lines used:
	 * SPI SDI (MOSI)	- data output to radio chip
	 * SPI SDO (MISO)	- data input from radio chip
	 * SPI SCLK			- clock source
	 * nSEL pin on GPIO	- transmission switch (0 = ON, 1 = OFF)
	 */
	radioSpi = spi;
	nselGpio = gpio;
	nselPin = pin;
}

uint8_t radioCmd(const uint8_t, rw, const uint8_t address, const uint8_t data) {
	/* Command consists of 16-bit word of below structure starting with MSB:
	 * RW:		1-bit operation type (1 = WRITE, 0 = READ)
	 * A6 - A0: 7-bit address
	 * D7 - D0: 8-bit data
	 */
}
