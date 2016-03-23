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
 * Creation date: 06/03/2013
 */

#ifndef _PANSTAMP_H
#define _PANSTAMP_H

#include "cc430core.h"
#include "cc430radio.h"
#include "cc430rtc.h"
#include "wiring.h"
#include "storage.h"

/**
 * Default carrier frequency
 */
#define DEFAULT_CARRIER_FREQ   CFREQ_868

/**
 * Default working mode
 */
#define DEFAULT_WORKING_MODE  0  // Speed = 38 Kbps
//#define DEFAULT_WORKING_MODE  MODE_LOW_SPEED  // Speed = 4800 bps

/**
 * Alias
 */
#define cc1101 radio

/**
 * RTC definitions
 */
#define RTC_250MS    0x03   // Timer 2 prescaler = 32
#define RTC_500MS    0x04   // Timer 2 prescaler = 64
#define RTC_1S       0x05   // Timer 2 prescaler = 128
#define RTC_2S       0x06   // Timer 2 prescaler = 256
#define RTC_8S       0x07   // Timer 2 prescaler = 1024

/**
 * Frequency Hopping and Spread Spectrum (FHSS)
 */
#define FHSS_ENABLED 1
#ifdef FHSS_ENABLED
#define FHSS_DWELLING_TIME 100
#define FHSS_MAX_HOPS 50
#define FHSS_BURST_LENGTH 5
#endif


/**
 * Macros
 */
#define setPacketRxCallback(ptrFunc)     ccPacketReceived = ptrFunc

#define setHighTxPower()    radio.setTxPowerAmp(PA_LongDistance)
#define setLowTxPower()     radio.setTxPowerAmp(PA_LowPower)

#define GET_RANDOM()  core.getShortUID()

/**
 * Class: PANSTAMP
 * 
 * Description:
 * panStamp main class
 */
class PANSTAMP
{
  public:
    #ifdef FHSS_ENABLED
    /**
     * Current FHSS channel
     */
    uint8_t currentChannel;

    /**
     * Accumulative packet fro FHSS purposes
     */
    CCPACKET fhssPacket;
    #endif

    /**
     * Radio interface
     */
    CC430RADIO radio;
  
    /**
     * MCU core
     */
    CC430CORE core;

    /**
     * RTC module
     */
    CC430RTC rtc;

    /**
     * CC11XX packet received. Callback function
     */
    void (*ccPacketReceived)(CCPACKET *packet);

    /**
     * PANSTAMP
     *
     * Class constructor
     */
    PANSTAMP(void);

    /**
     * rxOn
     *
     * Enable RF reception
     */
    void rxOn(void);
    
    /**
     * rxOff
     *
     * Disable RF reception
     */
    void rxOff(void);

    /**
     * init
     * 
     * Initialize panStamp board
     *
     * @param freq Carrier frequency
     * @param mode Working mode (speed, ...)
     */
    void init(uint8_t freq=DEFAULT_CARRIER_FREQ, uint8_t mode=DEFAULT_WORKING_MODE);

    /**
     * reset
     * 
     * Reset panStamp
     */
    void reset(void);

    /**
     * sleep
     *
     * Enter LPM4
     */
    void sleep(void);

    /**
     * sleepSec
     *
     * put the MCU in sleep mode
     *
     * @param time Sleep time in seconds
     * @param source Source of interruption (RTCSRC_VLO or RTCSRC_XT1)
     */
    void sleepSec(uint16_t time, RTCSRC source=RTCSRC_XT1);

    /**
     * attachInterrupt
     * 
     * Declare custom ISR, to be called whenever a wireless packet is received
     * 
     * @param funct pointer to the custom function
     */
     inline void attachInterrupt(void (*funct)(CCPACKET*))
     {
       ccPacketReceived = funct;
     }
     
    /**
     * getVcc
     *
     * Read MCU's voltage supply
     *
     * @return voltage in mV
     */
     inline uint16_t getVcc(void)
     {
       return core.getVcc();
     }
     
    /**
     * wakeUp
     *
     * Wake from sleep mode
     */
     inline void wakeUp(void)
     {
       core.setNormalMode();
       radio.setRxState();
     }

    /**
     * enableWirelessBoot
     * 
     * Enable/Disable wireless bootloader when the module starts
     * 
     * @param ena true for enabling the wireless bootloader
     */
    inline void enableWirelessBoot(bool ena)
    {
      bool * ptr;
      ptr = (bool*)RAM_END_ADDRESS;
      *ptr = !ena;
    }

    /**
     * goToWirelessBoot
     *
     * Start wireless bootloader
     */
     inline void goToWirelessBoot(void)
     {
       // Enable wireless bootloader
       enableWirelessBoot(true);
       
       // Go to wireless boot address
       void (*p)(void);
       p = (void (*)(void))WIRELESS_BOOT_ADDR;
       (*p)(); 
     }

    #ifdef FHSS_ENABLED
    /**
     * startDwellingTimer
     *
     * Start FHSS dwelling timer
     */
    inline void startDwellingTimer(void)
    {
      TA0CCTL0 = CCIE;                          // CCR0 interrupt enabled
      TA0CCR0 = 32.767 * FHSS_DWELLING_TIME;    // Max count
      TA0CTL = TASSEL_1 + MC_1 + TACLR;         // ACLK, upmode, clear TAR
    }

    /**
     * stopDwellingTimer
     *
     * Stop FHSS dwelling timer
     */
    inline void stopDwellingTimer(void)
    {
      TA0CTL = MC_0;                            // Halt timer
    }
    #endif

    /**
     * sendData
     *
     * Transmit packet
     *
     * @param packet Packet to be transmitted. First byte is the destination address
     */
    inline void sendData(CCPACKET packet)
    {
      #ifdef FHSS_ENABLED

      CCPACKET tmpPacket;
      uint8_t i, nbOfBursts = packet.length / FHSS_BURST_LENGTH;
      uint8_t lengthOfLastBurst = packet.length % FHSS_BURST_LENGTH;

      for(i=0 ; i<nbOfBursts ; i++)
      {
        memcpy(tmpPacket.data, packet.data + i*FHSS_BURST_LENGTH, FHSS_BURST_LENGTH);
        tmpPacket.length = FHSS_BURST_LENGTH;
        radio.sendData(tmpPacket);
        currentChannel++;
        radio.setChannel(currentChannel);
      }
      if (lengthOfLastBurst > 0)
      {
        memcpy(tmpPacket.data, packet.data + i*FHSS_BURST_LENGTH, lengthOfLastBurst);
        tmpPacket.length = lengthOfLastBurst;
        radio.sendData(tmpPacket);
      }
      // Back to the initial hop
      currentChannel = CCDEF_CHANNR;
      radio.setChannel(currentChannel);
      #else
        radio.sendData(packet);
      #endif
    }
};

/**
 * Global PANSTAMP object
 */
extern PANSTAMP panstamp;

#endif

