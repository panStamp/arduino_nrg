/*
 * Copyright (c) 2015 panStamp <contact@panstamp.com>
 * 
 * This file is part of the panStamp project.
 * 
 * panStamp  is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * any later version.
 * 
 * panStamp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with panStamp; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301
 * USA
 * 
 * Author: Daniel Berenguer
 * Creation date: 10/09/2015
 *
 * This library is based on mjmeli's WS2812 library for MSP430:
 * https://github.com/mjmeli/MSP430-NeoPixel-WS2812-Library
 *
 * This library works only for panStamp NRG and other MSP430 platforms.
 * LED's are driven through the SPI MOSI pin.
 */

#include "ws2812.h"

/**
 * show
 *
 * Load colors to the whole strip
 */
void WS2812STRIP::show(void)
{
  noInterrupts();  // Disable interrupts
	
  // Load RGB code for every LED
  uint16_t i, j;
  for (i = 0 ; i < nbOfLeds ; i++)
  {
    uint8_t rgb[3] = {leds[i].green, leds[i].red, leds[i].blue};	// get RGB color for this LED

		// send green, then red, then blue
		for (j = 0; j < 3; j++)
    {
      uint8_t mask = 0x80;          // b1000000

      // check each of the 8 bits
      while(mask != 0)
      {
        //while (!(UCB0IFG & UCTXIFG));

        if (rgb[j] & mask)          // most significant bit first
          UCB0TXBUF = HIGH_CODE;    // send 1
        else
          UCB0TXBUF = LOW_CODE;     // send 0

        mask >>= 1;						      // check next bit
      }
    }
  }

	// send RES code for at least 50 us
	delayMicroseconds(50);

	interrupts();       	   // enable interrupts
}

