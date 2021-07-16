/*-----------------------------------------------------------------------------
 Copyright (C) 2020-2021 ETH Zurich, Switzerland, University of Bologna, Italy.
 All rights reserved.

 File:    uart_dma_pulp.c (deprecated)
 Author:  Vlad Niculescu      <vladn@iis.ee.ethz.ch>
 Date:    15.03.2021
-------------------------------------------------------------------------------*/

/**
 * Modified by Victor Freire <freiremelgiz@wisc.edu>
 * University of Wisconsin-Madison
 * Jul 7, 2021
 */

#include <string.h>
#include "aideck_uart_dma.h"
#include "nvicconf.h"

DMA_InitTypeDef  DMA_InitStructure;
static uint8_t bufferTx[64];

static void USART_Config(uint32_t baudrate, uint8_t *bufferRx, uint32_t BUFFERSIZE);

// Send size bytes of data via USARTx
void USART_Send(uint32_t size, uint8_t *data){
  for(int i=0; i<size; i++){
    while(!(USARTx->SR & USART_FLAG_TXE));
    USARTx->DR = (data[i] & 0x00FF);
  }
}

// Send size bytes of data via USARTx with DMA
void USART_DMA_Send(uint32_t size, uint8_t *data){
  // Wait for DMA to be free
  while(DMA_GetCmdStatus(USARTx_TX_DMA_STREAM) != DISABLE);
  // Copy data in bufferTx
  memcpy(bufferTx, data, size);
  DMA_InitStructure.DMA_BufferSize = size;
  // Init new DMA stream
  DMA_Init(USARTx_TX_DMA_STREAM, &DMA_InitStructure);
  // Enable the Transfer Complete Interrupt
  DMA_ITConfig(USARTx_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
  // Enable USART DMA TX Requests
  USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE);
  // Clear transfer complete
  USART_ClearFlag(USARTx, USART_FLAG_TC);
  // Enable DMA USART TX Stream
  DMA_Cmd(USARTx_TX_DMA_STREAM, ENABLE);
}

// Reset the counter of the DMA to start transfer at initial address
void USART_DMA_ResetCounter(const int remaining_bytes, void *ptr_start){
  // Disable DMA Transfer-Complete interrupt
  DMA_ITConfig(USARTx_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
  // Disable DMA and wait for it
  DMA_Cmd(USARTx_RX_DMA_STREAM, DISABLE);
  while(DMA_GetCmdStatus(USARTx_RX_DMA_STREAM) != DISABLE);
  // Disable transfer complete
  DMA_ClearITPendingBit(USARTx_RX_DMA_STREAM, USARTx_RX_DMA_FLAG_TCIF);
  // Update DMA Counter
  DMA_SetCurrDataCounter(USARTx_RX_DMA_STREAM, remaining_bytes);
  // Update memory read address
  USARTx_RX_DMA_STREAM->M0AR = (uint32_t)ptr_start;
  // Enable DMA Transfer-Complete interrupt
  DMA_ITConfig(USARTx_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
  // Clear USART transfer complete
  USART_ClearFlag(USARTx, USART_FLAG_TC);
  // Enable DMA
  DMA_Cmd(USARTx_RX_DMA_STREAM, ENABLE);
}


// Configure the DMA and start it
void USART_DMA_Start(uint32_t baudrate, uint8_t *bufferRx, uint32_t BUFFERSIZE)
{
  // Setup Communication
  USART_Config(baudrate, bufferRx, BUFFERSIZE);

  DMA_ITConfig(USARTx_RX_DMA_STREAM, DMA_IT_TC, ENABLE);

  // Enable DMA USART RX Stream
  DMA_Cmd(USARTx_RX_DMA_STREAM,ENABLE);

  // Enable USART DMA RX Requsts
  USART_DMACmd(USARTx, USART_DMAReq_Rx, ENABLE);

  // Clear DMA Transfer Complete Flags
  DMA_ClearFlag(USARTx_RX_DMA_STREAM,USARTx_RX_DMA_FLAG_TCIF);

  // Clear USART Transfer Complete Flags
  USART_ClearFlag(USARTx,USART_FLAG_TC);
  DMA_ClearFlag(USARTx_RX_DMA_STREAM, UART3_RX_DMA_ALL_FLAGS);

  // Interrupts
  NVIC_EnableIRQ(USARTx_DMA_RX_IRQn);
  // TX Interrupt
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = USARTx_DMA_TX_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_MID_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

static void USART_Config(uint32_t baudrate, uint8_t *bufferRx, uint32_t BUFFERSIZE)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  // Enable GPIO clock
  RCC_AHB1PeriphClockCmd(USARTx_TX_GPIO_CLK | USARTx_RX_GPIO_CLK, ENABLE);

  // Enable USART clock
  USARTx_CLK_INIT(USARTx_CLK, ENABLE);

  // Enable the DMA clock
  RCC_AHB1PeriphClockCmd(USARTx_DMAx_CLK, ENABLE);

  // Configure USART Rx as input floating
  GPIO_InitStructure.GPIO_Pin = USARTx_RX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);
  // Configure USART Tx as alternate function
  GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);

  // Connect USART pins to Crazyflie RX1 annd TX1 - USART3 in the STM32 */
  GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);
  GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF);

  // USARTx configuration
  USART_OverSampling8Cmd(USARTx, ENABLE);

  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* When using Parity the word length must be configured to 9 bits */
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USARTx, &USART_InitStructure);

  /* Configure RX DMA Initialization Structure */
  DMA_InitStructure.DMA_BufferSize = BUFFERSIZE ;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USARTx->DR)) ;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;

  /* Configure RX DMA */
  DMA_InitStructure.DMA_Channel = USARTx_RX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)bufferRx;
  DMA_Init(USARTx_RX_DMA_STREAM, &DMA_InitStructure);

  /* Configure TX DMA Initialization Structure */
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Channel = USARTx_TX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)bufferTx;
  DMA_Init(USARTx_TX_DMA_STREAM, &DMA_InitStructure);

  /* Enable USART */
  USART_Cmd(USARTx, ENABLE);
}


void __attribute__((used)) DMA1_Stream3_IRQHandler(void){
  // Stop and cleanup DMA Stream
  DMA_ITConfig(USARTx_TX_DMA_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(USARTx_TX_DMA_STREAM, USARTx_TX_DMA_FLAG_TCIF);
  USART_DMACmd(USARTx, USART_DMAReq_Tx, DISABLE);
  DMA_Cmd(USARTx_TX_DMA_STREAM, DISABLE);
}
