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
 * Creation date: 06/25/2014
 */

#include "cc430f5137.h"
#include "pins.h"
#include "wiring.h"

#define bit_pos(A) ((A) == 1u << 0 ? 0 \
: (A) == 1u << 1 ? 1 \
: (A) == 1u << 2 ? 2 \
: (A) == 1u << 3 ? 3 \
: (A) == 1u << 4 ? 4 \
: (A) == 1u << 5 ? 5 \
: (A) == 1u << 6 ? 6 \
: (A) == 1u << 7 ? 7 \
: 0)

/**
 * pinMode
 * 
 * Config pin mode
 * 
 * @param pin pin number
 * @param mode pin mode
 */
void pinMode(uint8_t pin, uint8_t mode) 
{
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);

	volatile uint8_t *dir = portDirRegister(port);
	volatile uint8_t *ren = portRenRegister(port);
	volatile uint8_t *out = portOutputRegister(port);
	volatile uint8_t *sel = portSelRegister(port);

  *sel &= ~bit;     // Unselect alternate function
   
  switch(mode)
  {
    case INPUT:
      *dir &= ~bit;
      break;
    case INPUT_PULLUP:
      *dir &= ~bit;
      *out |= bit;
      *ren |= bit;
      break;
    case INPUT_PULLDOWN:
      *dir &= ~bit;
      *out &= ~bit;
      *ren |= bit;
      break;
    case OUTPUT:
      *dir |= bit;
      break;
    default:
      break;
  }
}

/**
 * pinMode_int
 * 
 * Config pin mode with additional working modes
 * 
 * @param pin pin number
 * @param mode pin mode
 */
void pinMode_int(uint8_t pin, uint16_t mode)
{
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);

	volatile uint8_t *dir;
	volatile uint8_t *ren;
	volatile uint8_t *out;
	volatile uint8_t *sel;

	if (port == NOT_A_PORT) return;

	dir = portDirRegister(port);
	ren = portRenRegister(port);
	out = portOutputRegister(port);

	if (mode & OUTPUT) {
		*dir |= bit;
	} else {
		*dir &= ~bit;
		if (mode & INPUT_PULLUP) {
			*out |= bit;
			*ren |= bit;
		} else if (mode & INPUT_PULLDOWN) {
			*out &= ~bit;
			*ren |= bit;
		} else {
	        *ren &= ~bit;
		}
	}

#if (defined(P1SEL_) || defined(P1SEL) || defined(__MSP430_HAS_P1SEL__))
	sel = portSel0Register(port);	/* get the port function select register address */
	if (mode & PORT_SELECTION0) {
		*sel |= bit;
	} else {
		*sel &= ~bit;
	}
#endif
#if (defined(P1SEL2_) || defined(P1SEL2) || defined(__MSP430_HAS_P1SEL2__))
	sel = portSel2Register(port);	/* get the port function select register address */
	if (mode & PORT_SELECTION1) {
		*sel |= bit;
	} else {
		*sel &= ~bit;
	}
#endif
#if (defined(P1SEL0_) || defined(P1SEL0) || defined(__MSP430_HAS_P1SEL0__))
	sel = portSel0Register(port);	/* get the port function select register address */
	if (mode & PORT_SELECTION0) {
		*sel |= bit;
	} else {
		*sel &= ~bit;
	}
#endif
#if (defined(P1SEL1_) || defined(P1SEL1) || defined(__MSP430_HAS_P1SEL1__))
	sel = portSel1Register(port);	/* get the port function select register address */
	if (mode & PORT_SELECTION1) {
		*sel |= bit;
	} else {
		*sel &= ~bit;
	}
#endif
#if defined(__MSP430_HAS_PORT_MAPPING__)
	volatile uint8_t *pmreg;
	pmreg = portPMReg(port);

	if(pmreg == NOT_A_PIN) return;

	// Store current interrupt state, then disable all interrupts, to avoid that the port map is put into read only mode
	uint16_t globalInterruptState = __get_interrupt_state();
	__disable_interrupt();

	PMAPKEYID = PMAPKEY;
	PMAPCTL |= PMAPRECFG;
	*(pmreg + bit_pos(bit)) = (mode >> 8) & 0xff;
	// Make port map control read only by writing invalid password
	PMAPKEYID = 0x0;

	// Restore previous interrupt state
	__set_interrupt_state(globalInterruptState);
#endif
}

/**
 * digitalRead
 *
 * read binary state
 *
 * @param pin pin mumber
 * 
 * @return input state
 */
uint8_t digitalRead(uint8_t pin)
{
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	volatile uint8_t *input = portInputRegister(port);

  return (((*input) & bit) > 0);
}

/**
 * digitalWrite
 *
 * set binary state
 *
 * @param pin pin mumber
 * @param state output state
 */
void digitalWrite(uint8_t pin, uint8_t state)
{
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	volatile uint8_t *out = portOutputRegister(port);
   
  if (state)
    *out |= bit;
  else
    *out &= ~bit;
}

