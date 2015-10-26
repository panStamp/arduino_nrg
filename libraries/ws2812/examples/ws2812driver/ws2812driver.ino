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
 */

#include "SPI.h"
#include "ws2812.h"

uint8_t initialRed=0, initialGreen=0, initialBlue=0;

// LED strip object (LED's are driven through the SPI MOSI pin)
WS2812STRIP strip(60);

void setup()
{
	//WDTCTL = WDTPW + WDTHOLD;				// Stop WDT
  strip.begin();
}

uint16_t i;

void loop()
{
  showGo(initialRed, initialGreen, initialBlue);
  showBack(initialRed, initialGreen, initialBlue);
}

void showGo(uint8_t r, uint8_t g, uint8_t b)
{
  uint16_t i;

  for (i = 0; i < strip.nbOfLeds; i++)
  {
    strip.setLEDColor(i, r, g, b);
    r += 3;
    g -= 2;
    b += 1;

    strip.show();

    //delayMicroseconds(6000);  // We can not use delay() here in order to avoid interrupts
    delay(20);
  }

  initialRed = r;
  initialGreen = g;
  initialBlue = b;
}

void showBack(uint8_t r, uint8_t g, uint8_t b)
{
  uint16_t i;

  for (i = strip.nbOfLeds; i > 0; i--)
  {
    strip.setLEDColor(i, r, g, b);
    r += 3;
    g -= 2;
    b += 1;

    strip.show();

    //delayMicroseconds(6000);  // We can not use delay() here in order to avoid interrupts
    delay(20);
  }

  initialRed = r;
  initialGreen = g;
  initialBlue = b;
}

