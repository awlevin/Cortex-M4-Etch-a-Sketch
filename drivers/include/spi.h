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

/************************************************************************/
/* FILE NAME    - SPI.h                                                 */
/* AUTHOR       - Joe Krachey                                           */
/* DATE CREATED - 24-Jun-2013                                           */
/* DESCRIPTION  - Header file for SPI.c                                 */
/*                                                                      */
/* (c) ECE Department, University of Wisconsin - Madison                */
/************************************************************************/
#ifndef __ECE453_SPI_H__
#define __ECE453_SPI_H__

#include "driver_defines.h"


//*****************************************************************************
// Function Prototypes
//*****************************************************************************

//*****************************************************************************
// Configure the given SPI peripheral to be 5Mhz and the given SPI mode
//*****************************************************************************
bool initialize_spi( uint32_t base_addr, uint8_t spi_mode, uint32_t cpsr);


//*****************************************************************************
// Transmits the array of bytes found at txData to the specified SPI peripheral
// The number of bytes transmitted is determined by numBytes.
//
// The data received by the SPI ternimal is placed in an array of bytes 
// starting at the address found at rxData
//*****************************************************************************
void spiTx(uint32_t base, uint8_t *tx_data, int num_bytes, uint8_t *rx_data);

#endif
