#ifndef SPI_H
#define SPI_H
#include "stm32f4xx_spi.h"
#include "misc.h"
#include "stm32f4xx.h"

#define SPI_PERIPH_MAX 3
#define SPI_TX_MAX 10
#define SPI_RX_MAX 10
#define CHECK_SPI1_IRQ(SPI_I2S_IT,handle) if (SPI_I2S_GetITStatus(SPI1,(SPI_I2S_IT))==SET) goto handle;

typedef enum {SPI1_Periph, SPI2_Periph, SPI3_Periph}eSPIperiph;

typedef struct
{
  eSPIperiph SPI_Id;
  unsigned char Tx_Buffer[SPI_TX_MAX];
  unsigned char Rx_Buffer[SPI_RX_MAX];
  void(*rxCallback)();
  void(*txCallback)();
}SPI_periph;

extern SPI_TypeDef * SPI_Array[SPI_PERIPH_MAX];
extern DMA_TypeDef* SPI_DMA_Array [SPI_PERIPH_MAX];
extern DMA_Stream_TypeDef* SPI_DMA_Tx_Stream [SPI_PERIPH_MAX];

SPI_periph SPI_Configure(eSPIperiph spiId,unsigned char *Tx_Buffer, unsigned char* Rx_Buffer,void(*rxCallback)(),void(*txCallback)());
void SPI_Send (SPI_periph *spiPeriph, unsigned char* txBuff);
/**
NOTE : This macro is only used in case of all the pins of 
the SPI are in the same GPIO port
***********************************************************/
#define SPI_ENABLE_GPIO_PIN(GPIOx, MOSI, MISO, SCK, CS)     GPIO_InitTypeDef GPIO_InitStruct;\
                                                            GPIO_InitStruct.GPIO_Pin = (MOSI)|(MISO)|(SCK)|(CS);\
                                                            GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;\
                                                            GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;\
                                                            GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;\
                                                            GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;\
                                                            GPIO_Init((GPIOx), &GPIO_InitStruct);\

#define SPI_AF_PIN_CONFIG(SPIx,GPIOx,MOSI,MISO,SCK)         GPIO_PinAFConfig((GPIOx), (MOSI), GPIO_AF_##SPIx);\
                                                            GPIO_PinAFConfig((GPIOx), (MISO), GPIO_AF_##SPIx);\
                                                            GPIO_PinAFConfig((GPIOx), (SCK), GPIO_AF_##SPIx);\

#define SPI_SET_CS_HIGH(GPIOx,CS)                         GPIOx->BSRRL |= CS;

/**
SPI Buffer
***********************************/
extern uint8_t spiTxBuff[2][SPI_TX_MAX];
extern uint8_t spiRxBuff[SPI_RX_MAX];

typedef void (*tSPI_Callback)(void);

/**
SPI Functions
********************************/
extern void _SPI1_interrupt_Enable();
extern void SPI1_Configure();
extern void SPI1_Start();
extern void SPI1_Send(uint8_t *txBuff,int length,tSPI_Callback fct);
extern void SPI1_Receive();
extern void SPI1_Stop(); 
extern void SPI1_DMA2_Enable_TC_Interrupts();
extern void SPI1_DMA2_Configure();
extern void SPI1_DMA2_Start();
extern void SPI1_DMA2_Stop();

/**
SPI Callback
*******************************/
extern void spi_TxCallback();
extern void spi_RxCallback();

#endif