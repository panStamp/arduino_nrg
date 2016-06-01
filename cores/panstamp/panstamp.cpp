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

#include "panstamp.h"

#ifdef FHSS_ENABLED
bool resetFHSS = false;

uint8_t PANSTAMP::hopSequence[] = {22,32,25,34,41,19,43,23,9,30,7,39,46,2,42,5,8,13,24,3,40,11,35,38,45,
                                   33,10,26,1,28,18,15,21,12,48,50,27,44,37,36,20,14,47,6,16,49,17,29,4,31};

bool newPacket = true;

void endOfReception(void);
#endif

/**
 * radioISR
 *
 * Radio interrupt routine
 */
__attribute__((interrupt(CC1101_VECTOR)))
void radioISR(void)
{
  unsigned int coreIntSource = RF1AIV;            // Radio Core      interrupt register

  // Radio Core interrupt
  if(coreIntSource)
  {
    // Check for SYNC interrupt
    if(coreIntSource == RF1AIV_RFIFG9)
    {
      if(MRFI_SYNC_PIN_INT_IS_ENABLED())
      {
        static CCPACKET ccPacket;

        /*  clear the sync pin interrupt, run sync pin ISR */
        /*
         *  NOTE!  The following macro clears the interrupt flag but it also *must*
         *  reset the interrupt capture.  In other words, if a second interrupt
         *  occurs after the flag is cleared it must be processed, i.e. this interrupt
         *  exits then immediately starts again.  Most microcontrollers handle this
         *  naturally but it must be verified for every target.
         */
        MRFI_CLEAR_SYNC_PIN_INT_FLAG();
        MRFI_DISABLE_SYNC_PIN_INT();

        // Any packet waiting to be read?
        if (panstamp.radio.receiveData(&ccPacket) > 0)
        {
          #ifdef FHSS_ENABLED
          // Stop FHSS dwelling timer
          panstamp.stopDwellingTimer();
    
          // Reset FHSS channel and buffer?
          if (resetFHSS)
          {
            resetFHSS = false;
            panstamp.fhssPacket.length = 0;
          }
          #endif

          // Is CRC OK?
          if (ccPacket.crc_ok)
          {
            #ifdef FHSS_ENABLED
            if (panstamp.getCurrentChannel() < FHSS_MAX_HOPS)
            {
              panstamp.currentChannelIndex++;

              if ((panstamp.fhssPacket.length + ccPacket.length) < CCPACKET_DATA_LEN)
              {
                memcpy(panstamp.fhssPacket.data + panstamp.fhssPacket.length, ccPacket.data, ccPacket.length);
                panstamp.fhssPacket.length += ccPacket.length;

                // First burst of packet?
                if (newPacket)
                {
                  newPacket = false;
                  panstamp.fhssPacket.lqi = ccPacket.lqi;
                  panstamp.fhssPacket.rssi = ccPacket.rssi;
                }

                // Burst length = max allowed length?
                if (ccPacket.length == FHSS_BURST_LENGTH)
                {
                  // OK, there is probably another burst that is coming
                  panstamp.radio.setChannel(panstamp.getCurrentChannel());
                  panstamp.startDwellingTimer();
                }
                else // ccPacket.length < FHSS_BURST_LENGTH // this means end of packet transmission
                {
                  endOfReception();
                }
              }
            }
            else  // Back to the initial hop
            {
              panstamp.currentChannelIndex = 0;
              panstamp.radio.setChannel(panstamp.getCurrentChannel());
              panstamp.fhssPacket.length = 0;
            }

            #else
            if (panstamp.ccPacketReceived != NULL)
              panstamp.ccPacketReceived(&ccPacket);
            #endif
          }
        }

        MRFI_ENABLE_SYNC_PIN_INT();
      }
    }
    // Check for RF_RDY (Event1 WOR) interrupt
    else if(coreIntSource == RF1AIV_RFIFG14)
    {
      RF1AIE |= BIT9 + BIT1;
      RF1AIFG &= ~(BIT9 + BIT1);
      RF1AIES |= BIT9; // Falling edge of RFIFG9
      panstamp.radio.setRxState();
      __bic_SR_register_on_exit(LPM3_bits);
    }
  }
}

#ifdef FHSS_ENABLED
/**
 * endOfReception
 *
 * End of FHSS reception. finish transmission and process packet
 * received
 */
void endOfReception(void)
{
  panstamp.stopDwellingTimer();

  // Dwelling time is over. Reset FHSS channel and buffer
  newPacket = true;
  resetFHSS = true;
  panstamp.currentChannelIndex = 0;
  panstamp.radio.setChannel(panstamp.getCurrentChannel());

  // Any packet received?
  if (panstamp.fhssPacket.length > 0)
  {
    // Call user function, if any
    if (panstamp.ccPacketReceived != NULL)
      panstamp.ccPacketReceived(&panstamp.fhssPacket);
  }
}

/**
 * DWELLING_TIMER_ISR
 * 
 * TimerA 0 ISR function - Dwelling timer ISR
 */
__attribute__((interrupt(TIMER0_A0_VECTOR)))
void DWELLING_TIMER_ISR(void)
{
  endOfReception();
  panstamp.radio.setRxOffState(); // Enter idle state
  panstamp.radio.setRxOnState();  // Back to Rx state
}
#endif

/**
 * PANSTAMP
 *
 * Class constructor
 */
PANSTAMP::PANSTAMP(void)
{
  ccPacketReceived = NULL;

  #ifdef FHSS_ENABLED
  currentChannelIndex = 0;
  fhssPacket.length = 0;
  #endif
}

/**
 * init
 * 
 * Initialize panStamp board
 * 
 * @param freq Carrier frequency
 * @param mode Working mode (speed, ...)
 */
void PANSTAMP::init(uint8_t freq, uint8_t mode) 
{
  // Disable wireless bootloader
  enableWirelessBoot(false);
  
  // Initialize MCU core
  core.init();

  // Initialize onboard LED pin
  INIT_ONBOARD_LED();

  // Initialize internal ACC power pin
  #if defined(__NRG_VERSION_1_0__) || defined(__NRG_VERSION_1_1__)
  INIT_ACC_POWER();
  #endif

  // Setup radio interface
  radio.init(freq, mode);

  delayMicroseconds(50);

  #ifdef FHSS_ENABLED
  radio.setChannel(getCurrentChannel());
  #endif

  delayMicroseconds(50);
}

/**
 * rxOn
 *
 * Enable RF reception
 */
void PANSTAMP::rxOn(void)
{
  MRFI_ENABLE_SYNC_PIN_INT();
}

/**
 * rxOff
 *
 * Disable RF reception
 */
void PANSTAMP::rxOff(void)
{
  MRFI_DISABLE_SYNC_PIN_INT();
}

/**
 * sleep
 *
 * Enter LPM4
 */
void PANSTAMP::sleep(void)
{
  // Power down radio
  radio.setPowerDownState();
  
  // If RTC calendar is running
  if (rtc.calendarIsRunning)
    core.setLowPowerMode();        // Enter LPM3
  else
    core.setLowPowerMode(true);    // Enter LPM4
  
  // Wake-up
  wakeUp();
}

/**
 * sleepSec
 *
 * put the MCU in sleep mode
 *
 * @param time Sleep time in seconds
 * @param source Source of interruption (RTCSRC_VLO or RTCSRC_XT1)
 */
void PANSTAMP::sleepSec(uint16_t time, RTCSRC source)
{ 
  if (time == 0)
    return;
   
  // Power down radio
  radio.setPowerDownState();

  core.delayClockCycles(0xFFFF);

  // Sleep
  rtc.sleep(time, source);
   
  // Wake-up radio
  radio.setRxState();
}

/**
 * sendData
 *
 * Transmit packet
 *
 * @param packet Packet to be transmitted. First byte is the destination address
 *
 * @return
 *  True if the transmission succeeds
 *  False otherwise
 */
bool PANSTAMP::sendData(CCPACKET packet)
{
  #ifdef FHSS_ENABLED
  CCPACKET tmpPacket;
  uint8_t i, nbOfBursts = packet.length / FHSS_BURST_LENGTH;
  uint8_t lengthOfLastBurst = packet.length % FHSS_BURST_LENGTH;

  for(i=0 ; i<nbOfBursts ; i++)
  {
    memcpy(tmpPacket.data, packet.data + i*FHSS_BURST_LENGTH, FHSS_BURST_LENGTH);
    tmpPacket.length = FHSS_BURST_LENGTH;
    if (!radio.sendData(tmpPacket))
      return false;
    currentChannelIndex++;
    radio.setChannel(getCurrentChannel());
  }
  if (lengthOfLastBurst > 0)
  {
    memcpy(tmpPacket.data, packet.data + i*FHSS_BURST_LENGTH, lengthOfLastBurst);
    tmpPacket.length = lengthOfLastBurst;
    if (!radio.sendData(tmpPacket))
      return false;
  }

  // Back to the initial hop
  currentChannelIndex = 0;
  radio.setChannel(getCurrentChannel());
  return true;

  #else
    return radio.sendData(packet);
  #endif
}

/**
 * reset
 * 
 * Reset panStamp
 */
void PANSTAMP::reset(void)
{
  WDTCTL = 0;
  while (1) {}
}
   
/**
 * Pre-instantiate PANSTAMP object
 */
PANSTAMP panstamp;

