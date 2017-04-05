// Copyright (c) 2014, Joe Krachey
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

#include "spi.h"

/****************************************************************************
 * This routine transmits a character out the SPI1 port.
 ****************************************************************************/
bool 
spiVerifyBaseAddr(uint32_t base)
{
  if ( base == SSI0_BASE ||
        base == SSI1_BASE ||
        base == SSI2_BASE ||
        base == SSI3_BASE
  )
  {
    return true;
  }
  else
  {
    return false;
  }
}

//*****************************************************************************
// Configure the given SPI peripheral to be 5Mhz and the given SPI mode
//*****************************************************************************
bool initialize_spi( uint32_t base_addr, uint8_t spi_mode, uint32_t cpsr)
{
    SSI0_Type *mySSI = (SSI0_Type *)base_addr;
  
    // Validate that a correct base address has been passed
    // Turn on the Clock Gating Register
    switch (base_addr) 
    {
      case SSI0_BASE :
      {
          SYSCTL->RCGCSSI |= SYSCTL_RCGCSSI_R0;
          while ((SYSCTL->PRSSI & SYSCTL_PRSSI_R0) == 0){}    /* wait until SSI is ready */
          break;
      }
      case SSI1_BASE :
      {
          SYSCTL->RCGCSSI |= SYSCTL_RCGCSSI_R1;
          while ((SYSCTL->PRSSI & SYSCTL_PRSSI_R1) == 0){}    /* wait until SSI is ready */
          break;
      }
      case SSI2_BASE :
      {
          SYSCTL->RCGCSSI |= SYSCTL_RCGCSSI_R2;
          while ((SYSCTL->PRSSI & SYSCTL_PRSSI_R2) == 0){}    /* wait until SSI is ready */
          break;
      }
      case SSI3_BASE :
      {
          SYSCTL->RCGCSSI |= SYSCTL_RCGCSSI_R3;
          while ((SYSCTL->PRSSI & SYSCTL_PRSSI_R3) == 0){}    /* wait until SSI is ready */
          break;
      }
      default:
      {
          return false;
      }
    }
    
    // ************* ADD CODE *********************** //
    // Disable the SSI interface (Set entire register to 0).

    
    // ************* ADD CODE *********************** //
    // Assume that we have a 50MHz clock
    // FSSIClk = FSysClk / (CPSDVSR * (1 + SCR))
    // Use the cpsr variable to set the CPSDVSR bits.  Set the SCR CR0 to 0.


    // ************* ADD CODE *********************** //
    // Configure SPI control for freescale format, data width of 8 bits

    
    // ************* ADD CODE *********************** //
    // Configure the SPI MODE in CR0
    switch (spi_mode)
    {
      case 0:
      {
        mySSI->CR0 |=  0;
        break;
      }
      case 1:
      {
        mySSI->CR0 |=  0;
        break;
      }
      case 2:
      {
        mySSI->CR0 |=  0;
        break;
      }
      case 3:
      {
        mySSI->CR0 |=  0;
        break;
      }
    }
    
    // ************* ADD CODE *********************** //
    //Enable SSI peripheral in master mode
    

  return true;
}


//*****************************************************************************
// Transmits the array of bytes found at tx_data to the specified SPI peripheral
// The number of bytes transmitted is determined by num_bytes.
//
// The data received by the SPI ternimal is placed in an array of bytes 
// starting at the address found at rx_data
//*****************************************************************************
void spiTx(uint32_t base, uint8_t *tx_data, int num_bytes, uint8_t *rx_data)
{
  uint8_t count = 0;
  SSI0_Type *mySSI = (SSI0_Type *) base;
  
  // Wait until the transmit is finished
  while((mySSI->SR & SSI_SR_TFE)!= 1){};

  // Disable the SSI interface
  mySSI->CR1 &= ~SSI_CR1_SSE;
    
  // Fill the Transmit FIFO  
  while((mySSI->SR & SSI_SR_TNF)!= 0 && (count < num_bytes) )
  {
    // Send out the first byte
    mySSI->DR = *tx_data; 
    tx_data++;
    count++;
  }
                  
  //Enable SSI
  mySSI->CR1 |= SSI_CR1_SSE;
  
  for( count = 0; count < num_bytes; count++)
  {
    // Wait until the recieve has finished  
    while((mySSI->SR & SSI_SR_RNE)==0){};// wait until response

    // Store the results  
    *rx_data =  mySSI->DR;
    rx_data++;
  }
}
