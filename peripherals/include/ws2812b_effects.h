// Copyright (c) 2016, Joe Krachey
// All rights reserved.
//
// Redistribution and use in binary form, with or without modification, 
// are permitted provided that the following conditions are met:
//
// 1. Redistributions in binary form must reproduce the above copyright 
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

#ifndef __WS2812B_EFFECTS_H__
#define __WS2812B_EFFECTS_H__
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "driver_defines.h"
#include "gpio_port.h"

#define NUMBER_LEDS     8
#define RAINBOW_SEQ_LEN 12
#define DIV_FACTOR      2

#define WS2812B_GPIO_BASE     GPIOD_BASE
#define WS2812B_GPIO_ADDR     (GPIOD_BASE  + 0x3FC)
#define WS2812B_GPIO_PIN      PD7
#define WS2812B_GPIO_PIN_NUM  7

typedef struct WS2812B_t WS2812B_t;

struct WS2812B_t
{
     uint8_t green;
     uint8_t red;
     uint8_t blue;
}__attribute__ ((packed)); 

bool ws2812b_init(void);
void ws2812b_leds_rainbow(void);

/********************************************************************************
* Summary:
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/ 
void ws2812b_leds_off(void);
  
#endif

