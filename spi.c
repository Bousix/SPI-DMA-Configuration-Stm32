#include "spi.h"

uint8_t spiTxBuff[2][SPI_TX_MAX] = {
                                    {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x10},
                                    {0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x1F,0x2F,0x3F,0x4F}
                                   };
uint8_t spiRxBuff[SPI_RX_MAX];
static volatile int spiRxCount= 0;

/************************************************************
SPI DMA functions
*************************************************************/
/* configure pins used by SPI1
         * PA4 = NSS
	 * PA5 = SCK
	 * PA6 = MISO
	 * PA7 = MOSI
*/

void SPI1_Configure()
{
   SPI_InitTypeDef SPI_InitStruct; 
  
  _SPI1_Pin_Config();
  _SPI1_interrupt_Enable();
	
  /* configure SPI1 in Mode 0 
   * CPOL = 0 --> clock is low when idle
   * CPHA = 0 --> data is sampled at the first edge
   */
  SPI_StructInit(&SPI_InitStruct); // set default settings 
  SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     // transmit in master mode, NSS pin has to be always high
  SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;        // clock is low when idle
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;      // data sampled at first edge
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft ; // set the NSS management to internal and pull internal NSS high
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // SPI frequency is APB2 frequency / 4
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
  SPI_Init(SPI1, &SPI_InitStruct);
}

void _SPI1_interrupt_Enable()
{
  NVIC_InitTypeDef NVIC_InitStructure;
   
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  SPI_I2S_ITConfig(SPI1,SPI_I2S_IT_RXNE,ENABLE);
}

void _SPI1_Pin_Config()
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5|GPIO_Pin_4;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  // connect SPI1 pins to SPI alternate function
  //GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI1);
  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
  
  //Set chip select high 
  GPIOA->BSRRL |= GPIO_Pin_4; // set PE4 high
	
  // enable peripheral clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
}



void SPI1_Send(uint8_t *txBuff,int length,tSPI_Callback fct)
{
   SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
   SPI_Cmd(SPI1, ENABLE);
   SPI1_DMA2_Start(txBuff);
}

void SPI1_Stop()
{
  SPI_Cmd(SPI1, DISABLE);
}

void SPI1_IRQHandler()
{
    if (SPI1->SR & SPI_I2S_FLAG_RXNE)
    {
      spiRxBuff[spiRxCount] = SPI_I2S_ReceiveData(SPI1);
      spiRxCount = (spiRxCount+1)%10;
    }
}

/**
DMA Related Functions
******************************************************************/
/**
DMA2
Channel3, Stream2: SPI1_RX
Channel3, Stream5: SPI1_TX
*/

void SPI1_DMA2_Configure()
{
  DMA_InitTypeDef DMA_InitStructure;   
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

  DMA_StructInit(&DMA_InitStructure);

  DMA_InitStructure.DMA_Channel = DMA_Channel_3;
  DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t) &(SPI1->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr  = (uint32_t) &spiTxBuff;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize  = SPI_TX_MAX;
  DMA_InitStructure.DMA_PeripheralInc  = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc  = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize  = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode  = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority  = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode  = DMA_FIFOMode_Disable;

  DMA_Init(DMA2_Stream5, &DMA_InitStructure);
  _SPI1_DMA2_Enable_TC_Interrupts();
  return;  
}

void SPI1_DMA2_Start(unsigned char * buff)
{
  DMA2_Stream5->M0AR = (uint32_t)buff;
  DMA_Cmd(DMA2_Stream5, ENABLE);
  return;
}

void _SPI1_DMA2_Enable_TC_Interrupts()
{
  NVIC_InitTypeDef NVIC_InitStructure;
   
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE);
  return;
}

void _SPI1_DMA2_Stop()
{
  DMA_ClearFlag(DMA2_Stream5,DMA_FLAG_FEIF2|DMA_FLAG_DMEIF2|DMA_FLAG_TEIF2|DMA_FLAG_HTIF2|DMA_FLAG_TCIF2);
  DMA_Cmd(DMA2_Stream5, DISABLE);
  return;
}