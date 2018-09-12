/**
 * Copyright (c) 2014 panStamp <contact@panstamp.com>
 * 
 * This file is part of the panStamp project.
 * 
 * panStamp  is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * any later version.
 * 
 * panStamp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with panStamp; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 
 * USA
 * 
 * Author: Daniel Berenguer
 * Creation date: 12/02/2014
 */

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

//#include <Arduino.h>
#include "cc430spi.h"

#define SPI_MODE0 0x02
#define SPI_MODE1 0x00
#define SPI_MODE2 0x03
#define SPI_MODE3 0x01


class SPISettings
{
  private:
    uint32_t clockFreq;
    uint8_t dataMode;
    uint32_t bitOrder;

  public:
    /**
     * Class constructor
     */
    inline SPISettings(uint32_t clock, uint8_t bOrder, uint8_t dMode)
    {
      clockFreq = clock;
      bitOrder = bOrder;
      dataMode = dMode;
    }

  friend class SPIClass;
};

class SPIClass
{
  private:
    /**
     * Low level SPI object
     */
    CC430SPI spiPort;
    
  public:
    /**
     * Initialize the SPI library
     */
    inline void begin()
    {
      spiPort.begin();
    }

    /**
     * Initialize the SPI library
     */
    inline void beginTransaction(SPISettings settings)
    {
      begin();
    }

    /**
     * transfer
     * 
     * Send single byte to SPI slave and read response
     *
     * @param data Byte to be sent
     * 
     * @return byte returned by slave
     */
    inline uint8_t transfer(uint8_t data)
    {
      return spiPort.transfer(data);
    }

    /**
     * write
     * 
     * Send data buffer to SPI slave
     *
     * @param data Buffer to be sent
     * @param len Amount of bytes to be transferred
     *
     * @return Amount of bytes transmitted
     */
    void inline transfer(uint8_t *buf, uint8_t count)
    {
      spiPort.transfer(buf, count);
    }

    /**
     * Disable the SPI bus
     */
    inline void endTransaction()
    {
      end();
    }

    /**
     * Disable the SPI bus
     */
    inline void end()
    {
    }
    
    /**
     * Not implemented
     */
    inline void setClockDivider(uint8_t clockDiv)
    {
    }
};

extern SPIClass SPI;
#endif
