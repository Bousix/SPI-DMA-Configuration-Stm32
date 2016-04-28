#ifndef SPI_H
#define SPI_H
#include "stm32f4xx_spi.h"
#include "misc.h"
#include "stm32f4xx.h"

#define SPI_PERIPH_MAX  3
#define SPI_TX_MAX      10
#define SPI_RX_MAX     10

typedef void (*tSPI_Callback)(void);
typedef enum {STATUS_SPI_CONFIG_OK     =0x01,
              STATUS_SPI_CONFIG_ERROR  =0x02, 
              STATUS_SPI_SEQ_ERROR     =0x03
              }eSPIStatus;

typedef struct
{
  uint8_t SPI_Id;
  unsigned char Tx_Buffer[SPI_TX_MAX];
  unsigned char Rx_Buffer[SPI_RX_MAX];
  unsigned char status; 
  tSPI_Callback rxCallback;
  tSPI_Callback txCallback;
}SPI_periph;

/**
SPI Buffer
***********************************/
extern uint8_t spiTxBuff[2][SPI_TX_MAX];
extern uint8_t spiRxBuff[SPI_RX_MAX];

/**
SPI Private Functions
********************************/
static void _SPI1_interrupt_Enable();
static void _SPI1_Pin_Config();

/**
SPI Public Functions
********************************/
extern void SPI1_Configure();
extern void SPI1_Send(uint8_t *txBuff,int length,tSPI_Callback fct);
extern void SPI1_Stop(); 

/**
SPI Callback
*******************************/
extern void spi_TxCallback();
extern void spi_RxCallback();

/**
DMA Public functions
*******************************/
extern void SPI1_DMA2_Configure();
extern void SPI1_DMA2_Start(unsigned char * buff);

/**
DMA Private functions
*******************************/
static void _SPI1_DMA2_Enable_TC_Interrupts();
static void _SPI1_DMA2_Stop();
#endif