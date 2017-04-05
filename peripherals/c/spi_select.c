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

#include "spi_select.h"

//*****************************************************************************
// Initialize the Pins used to determine which SPI device is active
//*****************************************************************************
bool  spi_select_init(void)
{
  //==============================
  // Configure the GPIO Pins as outputs
  //==============================
  if(gpio_enable_port(SPI_SELECT_0_GPIO_BASE) == false)
  {
    return false;
  }
  
  if(gpio_config_digital_enable(SPI_SELECT_0_GPIO_BASE, SPI_SELECT_0_PIN)== false)
  {
    return false;
  }

  if(gpio_config_enable_output(SPI_SELECT_0_GPIO_BASE, SPI_SELECT_0_PIN)== false)
  {
    return false;
  }
  
  if(gpio_enable_port(SPI_SELECT_1_GPIO_BASE) == false)
  {
    return false;
  }
  
  if(gpio_config_digital_enable(SPI_SELECT_1_GPIO_BASE, SPI_SELECT_1_PIN)== false)
  {
    return false;
  }

  if(gpio_config_enable_output(SPI_SELECT_1_GPIO_BASE, SPI_SELECT_1_PIN)== false)
  {
    return false;
  }
  
  spi_select(NORDIC);
  return true;
}

//*****************************************************************************
// Selects the currently active SPI device
//*****************************************************************************
void  spi_select(spi_device_t device)
{
    SPI_SELECT_0_PORT->DATA &= ~SPI_SELECT_0_PIN;
    SPI_SELECT_0_PORT->DATA |= device & SPI_SELECT_0_PIN;
    
    SPI_SELECT_1_PORT->DATA &= ~SPI_SELECT_1_PIN;
    SPI_SELECT_1_PORT->DATA |= device & SPI_SELECT_1_PIN;
}
