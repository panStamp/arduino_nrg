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
 */

#ifndef _WS2812_H
#define _WS2812_H

#include "SPI.h"

struct LEDSTRUCT
{
	uint8_t red;
	uint8_t green;
	uint8_t blue;
};

// Transmit codes
#define HIGH_CODE	(0xF0)			// b11110000
#define LOW_CODE	(0xC0)			// b11000000

// Maximum number of LED's
#define MAX_NUM_LEDS  200

class WS2812STRIP
{
  private:
    /**
     * Control pin
     */
    uint16_t ctrlPin;

    /**
     * SPI port
     */
    SPIClass spi;

    /**
     * LED information
     */
    LEDSTRUCT leds[MAX_NUM_LEDS];

  public:
    /**
     * Amount of leds
     */
    uint16_t nbOfLeds;

    /**
     * Class constructor
     * 
     * @param numLeds ANumber of LED's
     */
    inline WS2812STRIP(uint16_t numLeds)
    {
      nbOfLeds = numLeds;
    }

    /**
     * begin
     *
     * Start strip control
     */
    void inline begin(void)
    {
      spi.begin();
    	clear();
    }

    /**
     * clear
     *
     * Clear the color of all LEDs (make them black/off)
     */
    void inline clear(void)
    {
	    fill(0x00, 0x00, 0x00);  // black
    }

    /**
     * show
     *
     * Load colors to the whole strip and show
     */
    void show(void);

    /**
     * fill
     *
     * Fill the strip with a solid color. This will update the strip.
     */
    void inline fill(uint8_t r, uint8_t g, uint8_t b)
    {
      uint16_t i;

      for (i = 0; i < nbOfLeds; i++)
        setLEDColor(i, r, g, b);
      show(); // refresh strip
    }

    /**
     * setLEDColor
     *
     * Set the color for a given LED
     */
    void inline setLEDColor(uint8_t p, uint8_t r, uint8_t g, uint8_t b)
    {
      leds[p].red = r;
      leds[p].green = g;
      leds[p].blue = b;
    }
};

#endif

