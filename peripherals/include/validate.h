// Copyright (c) 2015, Joe Krachey
// All rights reserved.
//
// Redistribution and use in source or binary form, with or without modification, 
// are permitted provided that the following conditions are met:
//
// 1. Redistributions in source form must reproduce the above copyright 
//    notice, this list of conditions and the following disclaimer in 
//    the documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#ifndef __VALIDATE_H__
#define __VALIDATE_H__

#include <stdint.h>
#include <stdbool.h>

typedef enum {
  ICE_INTRO_TO_C,
  ICE_PC_BUFFER,
  ICE_GPIOF,
  ICE_GPIO_PORT,
  ICE_ADC,
  ICE_TIMER0,
  ICE_SYSTICK,
  ICE_UART_POLLING,
  ICE_UART_IRQ_RX,
  ICE_UART_IRQ_TX,
  ICE_SPI_LCD,
  ICE_SPI_NORDIC,
  ICE_I2C_FT6206,
  ICE_I2C_EEPROM
} ice_validate_t;

//*****************************************************************************
//*****************************************************************************
void initialize_serial_debug(void);

//*****************************************************************************
//*****************************************************************************
void get_string(char *string);

/****************************************************************************
 ****************************************************************************/
void put_char(char c);
	
//*****************************************************************************
//*****************************************************************************
void put_string(char *data);

//*****************************************************************************
//*****************************************************************************
char get_char(bool block);

//*****************************************************************************
//*****************************************************************************
bool validate_ice(ice_validate_t ice);

#endif 
