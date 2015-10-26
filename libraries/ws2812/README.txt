Basic routines for the WS2812 addressable RGB LED

This library works only for panStamp NRG and other MSP430 platforms.
LED's are driven through the SPI MOSI pin.

WS2812 LED's require driving voltages over 3.5 VDC. However, driving
them from 3.3V works most of the times. Simply understand that
driving these LED's out of spec may cause unexpected behaviors.

See the example for more information.
