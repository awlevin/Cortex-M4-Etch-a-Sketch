#ifndef __WIRELESS_H__
#define __WIRELESS_H__

#include <stdint.h>
#include <stdio.h>
#include "TM4C123GH6PM.h"

#include "gpio_port.h"
#include "spi.h"

//*****************************************************************************
// Fill out the #defines below to configure which pins are connected to
// the nordic wirless radio
//*****************************************************************************
#define   RF_GPIO_BASE       	GPIOA_BASE
#define   RF_SPI_BASE        	SSI0_BASE

#define   RF_CLK_PIN         	PA2
#define   RF_MISO_PIN        	PA4
#define   RF_MOSI_PIN        	PA5

#define   RF_SPI_CLK_PCTL_M  	GPIO_PCTL_PA2_M
#define   RF_SPI_MISO_PCTL_M  GPIO_PCTL_PA4_M
#define   RF_SPI_MOSI_PCTL_M  GPIO_PCTL_PA5_M

#define   RF_CLK_PIN_PCTL    	GPIO_PCTL_PA2_SSI0CLK
#define   RF_MISO_PIN_PCTL   	GPIO_PCTL_PA4_SSI0RX
#define   RF_MOSI_PIN_PCTL   	GPIO_PCTL_PA5_SSI0TX

#define   RF_CS_BASE        	GPIOA_BASE
#define   RF_CS_PIN          	PA3
#define   RF_CS_PORT         	GPIOA

#define   RF_CE_GPIO_BASE     GPIOD_BASE
#define   RF_CE_PIN           PD6
#define   RF_CE_PORT         	GPIOD

#define   RF_IRQ_GPIO_BASE    GPIOD_BASE
#define   RF_IRQ_PIN          PD3

#define   RF_PAYLOAD_SIZE     0x04
#define   RF_CHANNEL          0x02



#define NRF24L01_CONFIG_R                     0x00
#define NRF24L01_EN_AA_R                      0x01
#define NRF24L01_EN_RXADDR_R                  0x02
#define NRF24L01_SETUP_AW_R                   0x03
#define NRF24L01_SETUP_RETR_R                 0x04
#define NRF24L01_RF_CH_R                      0x05
#define NRF24L01_RF_SETUP_R                   0x06
#define NRF24L01_STATUS_R                     0x07
#define NRF24L01_OBSERVE_TX_R                 0x08
#define NRF24L01_RPD_R                        0x09
#define NRF24L01_RX_ADDR_P0_R                 0x0A
#define NRF24L01_RX_ADDR_P1_R                 0x0B
#define NRF24L01_RX_ADDR_P2_R                 0x0C
#define NRF24L01_RX_ADDR_P3_R                 0x0D
#define NRF24L01_RX_ADDR_P4_R                 0x0E
#define NRF24L01_RX_ADDR_P5_R                 0x0F
#define NRF24L01_TX_ADDR_R                    0x10
#define NRF24L01_RX_PW_P0_R                   0x11
#define NRF24L01_RX_PW_P1_R                   0x12
#define NRF24L01_RX_PW_P2_R                   0x13
#define NRF24L01_RX_PW_P3_R                   0x14
#define NRF24L01_RX_PW_P4_R                   0x15
#define NRF24L01_RX_PW_P5_R                   0x16
#define NRF24L01_FIFO_STATUS_R                0x17
#define NRF24L01_DYNPD_R                      0x1C
#define NRF24L01_FEATURE_R                    0x1D


#define  NRF24L01_CONFIG_MASK_RX_DR_N         ~( 0x1 << 6 )
#define  NRF24L01_CONFIG_MASK_TX_DS_N         ~( 0x1 << 5 )
#define  NRF24L01_CONFIG_MASK_MAX_RT_N        ~( 0x1 << 4 )
#define  NRF24L01_CONFIG_EN_CRC                ( 0x1 << 3 )
#define  NRF24L01_CONFIG_CRCO_2BYTES           ( 0x1 << 2 )
#define  NRF24L01_CONFIG_CRCO_1BYTES_N        ~( 0x1 << 2 )
#define  NRF24L01_CONFIG_PWR_UP                ( 0x1 << 1 )
#define  NRF24L01_CONFIG_PRIM_RX_PRX           ( 0x1 << 0 )
#define  NRF24L01_CONFIG_PRIM_RX_PTX            0x00

#define  NRF24L01_ENAA_P5                      ( 0x1 << 5 )
#define  NRF24L01_ENAA_P4                      ( 0x1 << 4 )
#define  NRF24L01_ENAA_P3                      ( 0x1 << 3 )
#define  NRF24L01_ENAA_P2                      ( 0x1 << 2 )
#define  NRF24L01_ENAA_P1                      ( 0x1 << 1 )
#define  NRF24L01_ENAA_P0                      ( 0x1 << 0 )

#define  NRF24L01_RXADDR_ERX_P5                ( 0x1 << 5 )
#define  NRF24L01_RXADDR_ERX_P4                ( 0x1 << 4 )
#define  NRF24L01_RXADDR_ERX_P3                ( 0x1 << 3 )
#define  NRF24L01_RXADDR_ERX_P2                ( 0x1 << 2 )
#define  NRF24L01_RXADDR_ERX_P1                ( 0x1 << 1 )
#define  NRF24L01_RXADDR_ERX_P0                ( 0x1 << 0 )

#define  NRF24L01_SETUP_AW_3_BYTES              0x01
#define  NRF24L01_SETUP_AW_4_BYTES              0x02
#define  NRF24L01_SETUP_AW_5_BYTES              0x03

#define  NRF24L01_SETUP_RETR_ARD_M                0xF0
#define  NRF24L01_SETUP_RETR_ARC_M                0x0F
#define  NRF24L01_SETUP_RETR_ARD_0250_US          0x00
#define  NRF24L01_SETUP_RETR_ARD_0500_US          0x10
#define  NRF24L01_SETUP_RETR_ARD_0750_US          0x20
#define  NRF24L01_SETUP_RETR_ARD_1000_US          0x30
#define  NRF24L01_SETUP_RETR_ARD_1250_US          0x40
#define  NRF24L01_SETUP_RETR_ARD_1500_US          0x50
#define  NRF24L01_SETUP_RETR_ARD_1750_US          0x60
#define  NRF24L01_SETUP_RETR_ARD_2000_US          0x70
#define  NRF24L01_SETUP_RETR_ARD_2250_US          0x80
#define  NRF24L01_SETUP_RETR_ARD_2500_US          0x90
#define  NRF24L01_SETUP_RETR_ARD_2750_US          0xA0
#define  NRF24L01_SETUP_RETR_ARD_3000_US          0xB0
#define  NRF24L01_SETUP_RETR_ARD_3250_US          0xC0
#define  NRF24L01_SETUP_RETR_ARD_3500_US          0xD0
#define  NRF24L01_SETUP_RETR_ARD_3750_US          0xE0
#define  NRF24L01_SETUP_RETR_ARD_4000_US          0xF0
#define  NRF24L01_SETUP_RETR_ARC_NO_RETRANS       0x00
#define  NRF24L01_SETUP_RETR_ARC_1                0x01
#define  NRF24L01_SETUP_RETR_ARC_2                0x02
#define  NRF24L01_SETUP_RETR_ARC_3                0x03
#define  NRF24L01_SETUP_RETR_ARC_4                0x04
#define  NRF24L01_SETUP_RETR_ARC_5                0x05
#define  NRF24L01_SETUP_RETR_ARC_6                0x06
#define  NRF24L01_SETUP_RETR_ARC_7                0x07
#define  NRF24L01_SETUP_RETR_ARC_8                0x08
#define  NRF24L01_SETUP_RETR_ARC_9                0x09
#define  NRF24L01_SETUP_RETR_ARC_10               0x0A
#define  NRF24L01_SETUP_RETR_ARC_11               0x0B
#define  NRF24L01_SETUP_RETR_ARC_12               0x0C
#define  NRF24L01_SETUP_RETR_ARC_13               0x0D
#define  NRF24L01_SETUP_RETR_ARC_14               0x0E
#define  NRF24L01_SETUP_RETR_ARC_15               0x0F

#define  NRF24L01_RF_CH_M                       0x3F

#define  NRF24L01_RF_SETUP_CONT_WAVE_M          ( 0x1 << 7 )
#define  NRF24L01_RF_SETUP_RF_DR_LOW_M          ( 0x1 << 5 )
#define  NRF24L01_RF_SETUP_PLL_LOCK_M           ( 0x1 << 4 )
#define  NRF24L01_RF_SETUP_RF_DR_HIGH_M         ( 0x1 << 3 )
#define  NRF24L01_RF_SETUP_RF_PWR_M             ( 0x3 << 1 )
#define  NRF24L01_RF_SETUP_RF_PWR_N18DB         ( 0x0 << 1 )
#define  NRF24L01_RF_SETUP_RF_PWR_N12DB         ( 0x1 << 1 )
#define  NRF24L01_RF_SETUP_RF_PWR_N6DB          ( 0x2 << 1 )
#define  NRF24L01_RF_SETUP_RF_PWR_0DB           ( 0x3 << 1 )
#define  NRF24L01_RF_SETUP_1_MBPS               0x00
#define  NRF24L01_RF_SETUP_2_MBPS               0x08
#define  NRF24L01_RF_SETUP_250_KBPS             0x20

#define NRF24L01_STATUS_RX_DR_M                 ( 0x1 << 6 )
#define NRF24L01_STATUS_TX_DS_M                 ( 0x1 << 5 )
#define NRF24L01_STATUS_MAX_RT_M                ( 0x1 << 4 )
#define NRF24L01_STATUS_RX_P_NO_M               ( 0x7 << 1 )
#define NRF24L01_STATUS_TX_FULL_M               ( 0x1 << 0 )
#define NRF24L01_STATUS_CLEAR_ALL               0xFF

#define NRF24L01_OBSERVE_TX_PLOS_CNT_M          0xF0
#define NRF24L01_OBSERVE_TX_ARC_CNT_M           0x0F

#define NRF24L01_RPD_M                          ( 0x1 << 0 )

#define NRF24L01_FIFO_STATUS_TX_REUSE_M         ( 0x1 << 6 )
#define NRF24L01_FIFO_STATUS_TX_FULL_M          ( 0x1 << 5 )
#define NRF24L01_FIFO_STATUS_TX_EMPTY_M         ( 0x1 << 4 )
#define NRF24L01_FIFO_STATUS_RX_FULL_M          ( 0x1 << 1 )
#define NRF24L01_FIFO_STATUS_RX_EMPTY_M         ( 0x1 << 0 )

#define NRF24L01_CMD_R_REGISTER                 0x00
#define NRF24L01_CMD_W_REGISTER                 0x20
#define NRF24L01_CMD_R_RX_PAYLOAD               0x61
#define NRF24L01_CMD_W_TX_PAYLOAD               0xA0
#define NRF24L01_CMD_FLUSH_TX                   0xE1
#define NRF24L01_CMD_FLUSH_RX                   0xE2
#define NRF24L01_CMD_REUSE_TX_PL                0xE3
#define NRF24L01_CMD_R_RX_PL_WID                0x60
#define NRF24L01_CMD_W_ACK_PAYLOAD              0xA8
#define NRF24L01_CMD_TX_PAYLOAD_NO_ACK          0xB0
#define NRF24L01_CMD_NOP                        0xFF


extern const char *wireless_error_messages[];

typedef enum {
  PTX  = 0x00,
  PRX  = 0x01
} wireless_mode_t;


typedef enum {
  NRF24L01_TX_SUCCESS,
  NRF24L01_TX_FIFO_FULL,
  NRF24L01_TX_PCK_LOST,
  NRF24L01_RX_SUCCESS,
  NRF24L01_RX_FIFO_EMPTY,
  NRF24L01_ERR
} wireless_com_status_t;

//*****************************************************************************
// Transmits 4 bytes of data to the remote device.
//*****************************************************************************
wireless_com_status_t 
wireless_send_32(
  bool      block,
  bool      retry,
  uint32_t   data
  );

//*****************************************************************************
// Receives 4 bytes of data from the remote board.  The user can optionally
// block until data arrives.
//*****************************************************************************
wireless_com_status_t 
  wireless_get_32(
  bool      blockOnEmpty,
  uint32_t  *data
  );


//*****************************************************************************
// Configures the Nordic nRF24L01+ to be in a point-to-point configuration.
// 
// my_id is a pointer to a 5-byte array that contains the id of the local
// nordic radio
//
// dest_id is a pointer to the 5-byte array that contains the id of the remote
// nordic radio.
//*****************************************************************************
bool wireless_configure_device( 
  uint8_t           *my_id,
  uint8_t           *dest_id
);

//*****************************************************************************
// Used to initialize the Nordic nRF24L01+ radio.  The GPIO pins and SPI 
// interface are both configured.
//
// Configuration Info
//		Fill out relevant information in boardUtil.h.  boardUtil.h defines 
//		how various peripherals are physically connected to the board.
//  
//*****************************************************************************
void wireless_initialize(void);

//*****************************************************************************
// Test Rx and Tx of the wireless radio
//*****************************************************************************
void wireless_test(void);

#endif
