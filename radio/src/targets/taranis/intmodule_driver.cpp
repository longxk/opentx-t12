/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "opentx.h"

void intmoduleStop()
{
  NVIC_DisableIRQ(INTMODULE_DMA_STREAM_IRQn);
  NVIC_DisableIRQ(INTMODULE_TIMER_CC_IRQn);
  INTMODULE_DMA_STREAM->CR &= ~DMA_SxCR_EN; // Disable DMA

#if defined(INTMODULE_USART)
  INTMODULE_USART_TX_DMA_STREAM->CR &= ~DMA_SxCR_EN; // Disable DMA

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = INTMODULE_TX_GPIO_PIN | INTMODULE_RX_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(INTMODULE_USART_GPIO, &GPIO_InitStructure);

  GPIO_ResetBits(INTMODULE_USART_GPIO, INTMODULE_TX_GPIO_PIN | INTMODULE_RX_GPIO_PIN);
#endif

  INTMODULE_TIMER->DIER &= ~(TIM_DIER_CC2IE | TIM_DIER_UDE);
  INTMODULE_TIMER->CR1 &= ~TIM_CR1_CEN;

  INTERNAL_MODULE_OFF();
}

#if defined(INTERNAL_MODULE_PPM)
void intmodulePpmStart()
{
  INTERNAL_MODULE_ON();

  GPIO_PinAFConfig(INTMODULE_TX_GPIO, INTMODULE_TX_GPIO_PinSource, INTMODULE_TIMER_TX_GPIO_AF);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = INTMODULE_TX_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(INTMODULE_TX_GPIO, &GPIO_InitStructure);

  INTMODULE_TIMER->CR1 &= ~TIM_CR1_CEN;
  INTMODULE_TIMER->PSC = INTMODULE_TIMER_FREQ / 2000000 - 1; // 0.5uS from 30MHz
  INTMODULE_TIMER->CCR1 = GET_MODULE_PPM_DELAY(INTERNAL_MODULE)*2;
  INTMODULE_TIMER->CCER = INTMODULE_TIMER_OUTPUT_ENABLE | (GET_MODULE_PPM_POLARITY(INTERNAL_MODULE) ? INTMODULE_TIMER_OUTPUT_POLARITY : 0); //     // we are using complementary output so logic has to be reversed here
  INTMODULE_TIMER->BDTR = TIM_BDTR_MOE;
  INTMODULE_TIMER->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0; // Force O/P high
  INTMODULE_TIMER->EGR = 1;
  INTMODULE_TIMER->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2PE; // PWM mode 1
  INTMODULE_TIMER->SR &= ~TIM_SR_CC2IF; // Clear flag
  INTMODULE_TIMER->ARR = 45000;
  INTMODULE_TIMER->CCR2 = 40000; // The first frame will be sent in 20ms
  INTMODULE_TIMER->DIER |= TIM_DIER_UDE | TIM_DIER_CC2IE;
  INTMODULE_TIMER->CR1 |= TIM_CR1_CEN;

  NVIC_EnableIRQ(INTMODULE_TIMER_DMA_STREAM_IRQn);
  NVIC_SetPriority(INTMODULE_TIMER_DMA_STREAM_IRQn, 7);
  NVIC_EnableIRQ(INTMODULE_TIMER_CC_IRQn);
  NVIC_SetPriority(INTMODULE_TIMER_CC_IRQn, 7);
}
#endif // defined(INTERNAL_MODULE_PPM)

void intmoduleSerialStart(uint32_t /*baudrate*/, uint32_t period_half_us, bool inverted)
{
  INTERNAL_MODULE_ON();

  GPIO_PinAFConfig(INTMODULE_TX_GPIO, INTMODULE_TX_GPIO_PinSource, INTMODULE_TX_GPIO_AF);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = INTMODULE_TX_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(INTMODULE_TX_GPIO, &GPIO_InitStructure);

  INTMODULE_TIMER->CR1 &= ~TIM_CR1_CEN;
  INTMODULE_TIMER->PSC = INTMODULE_TIMER_FREQ / 2000000 - 1; // 0.5uS from 30MHz
  INTMODULE_TIMER->CCER = INTMODULE_TIMER_OUTPUT_ENABLE | (inverted ? 0 : INTMODULE_TIMER_OUTPUT_POLARITY);
  INTMODULE_TIMER->BDTR = TIM_BDTR_MOE; // Enable outputs
  INTMODULE_TIMER->CCR1 = 0;
  INTMODULE_TIMER->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0; // Force O/P high
  INTMODULE_TIMER->EGR = 1; // Restart
  INTMODULE_TIMER->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0;
  INTMODULE_TIMER->ARR = period_half_us;
  INTMODULE_TIMER->CCR2 = 40000; // The first frame will be sent in 20ms
  INTMODULE_TIMER->SR &= ~TIM_SR_CC2IF; // Clear flag
  INTMODULE_TIMER->DIER |= TIM_DIER_UDE | TIM_DIER_CC2IE;
  INTMODULE_TIMER->CR1 |= TIM_CR1_CEN;

  NVIC_EnableIRQ(INTMODULE_DMA_STREAM_IRQn);
  NVIC_SetPriority(INTMODULE_DMA_STREAM_IRQn, 7);
  NVIC_EnableIRQ(INTMODULE_TIMER_CC_IRQn);
  NVIC_SetPriority(INTMODULE_TIMER_CC_IRQn, 7);
}

ModuleFifo intmoduleFifo;
#if defined(INTMODULE_USART)
ModuleFifo intmoduleFifo;

void intmoduleInvertedSerialStart(uint32_t baudrate)
{
  INTERNAL_MODULE_ON();

  // TX + RX Pins
  GPIO_PinAFConfig(INTMODULE_USART_GPIO, INTMODULE_TX_GPIO_PinSource, INTMODULE_USART_GPIO_AF);
  GPIO_PinAFConfig(INTMODULE_USART_GPIO, INTMODULE_RX_GPIO_PinSource, INTMODULE_USART_GPIO_AF);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = INTMODULE_TX_GPIO_PIN | INTMODULE_RX_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(INTMODULE_USART_GPIO, &GPIO_InitStructure);

  // UART config
  USART_DeInit(INTMODULE_USART);
  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(INTMODULE_USART, &USART_InitStructure);
  USART_Cmd(INTMODULE_USART, ENABLE);

  intmoduleFifo.clear();

  USART_ITConfig(INTMODULE_USART, USART_IT_RXNE, ENABLE);
  NVIC_SetPriority(INTMODULE_USART_IRQn, 6);
  NVIC_EnableIRQ(INTMODULE_USART_IRQn);
}

void intmoduleSendBuffer(const uint8_t * data, uint8_t size)
{
  DMA_InitTypeDef DMA_InitStructure;
  DMA_DeInit(INTMODULE_USART_TX_DMA_STREAM);
  DMA_InitStructure.DMA_Channel = INTMODULE_USART_TX_DMA_CHANNEL;
  DMA_InitStructure.DMA_PeripheralBaseAddr = CONVERT_PTR_UINT(&INTMODULE_USART->DR);
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_Memory0BaseAddr = CONVERT_PTR_UINT(data);
  DMA_InitStructure.DMA_BufferSize = size;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(INTMODULE_USART_TX_DMA_STREAM, &DMA_InitStructure);
  DMA_Cmd(INTMODULE_USART_TX_DMA_STREAM, ENABLE);
  USART_DMACmd(INTMODULE_USART, USART_DMAReq_Tx, ENABLE);
}

#define USART_FLAG_ERRORS (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)
extern "C" void INTMODULE_USART_IRQHandler(void)
{
  uint32_t status = INTMODULE_USART->SR;

  while (status & (USART_FLAG_RXNE | USART_FLAG_ERRORS)) {
    uint8_t data = INTMODULE_USART->DR;
    if (status & USART_FLAG_ERRORS) {
      intmoduleFifo.errors++;
    }
    else {
      intmoduleFifo.push(data);
    }
    status = INTMODULE_USART->SR;
  }
}
#endif

#if defined(PXX1)
void intmodulePxx1PulsesStart()
{
  INTERNAL_MODULE_ON();

  GPIO_PinAFConfig(INTMODULE_TX_GPIO, INTMODULE_TX_GPIO_PinSource, INTMODULE_TX_GPIO_AF);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = INTMODULE_TX_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(INTMODULE_TX_GPIO, &GPIO_InitStructure);

  INTMODULE_TIMER->CR1 &= ~TIM_CR1_CEN;
  INTMODULE_TIMER->PSC = INTMODULE_TIMER_FREQ / 2000000 - 1; // 0.5uS (2Mhz)
  INTMODULE_TIMER->CCER = INTMODULE_TIMER_OUTPUT_ENABLE | INTMODULE_TIMER_OUTPUT_POLARITY; // polarity, default low
  INTMODULE_TIMER->BDTR = TIM_BDTR_MOE; // Enable outputs
  INTMODULE_TIMER->CCR1 = 18;
  INTMODULE_TIMER->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0; // Force O/P high
  INTMODULE_TIMER->EGR = 1; // Restart
  INTMODULE_TIMER->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
  INTMODULE_TIMER->ARR = 45000;
  INTMODULE_TIMER->CCR2 = 40000; // The first frame will be sent in 20ms
  INTMODULE_TIMER->SR &= ~TIM_SR_CC2IF; // Clear flag
  INTMODULE_TIMER->DIER |= TIM_DIER_UDE | TIM_DIER_CC2IE;
  INTMODULE_TIMER->CR1 |= TIM_CR1_CEN;

  NVIC_EnableIRQ(INTMODULE_DMA_STREAM_IRQn);
  NVIC_SetPriority(INTMODULE_DMA_STREAM_IRQn, 7);
  NVIC_EnableIRQ(INTMODULE_TIMER_CC_IRQn);
  NVIC_SetPriority(INTMODULE_TIMER_CC_IRQn, 7);
}
#endif

#if defined(PXX1) && defined(INTMODULE_USART)
void intmodulePxx1SerialStart()
{
  intmoduleInvertedSerialStart(INTMODULE_PXX1_SERIAL_BAUDRATE);
}
#endif

void intmoduleSendNextFrame()
{
  switch (moduleState[INTERNAL_MODULE].protocol) {
#if defined(INTERNAL_MODULE_PPM)
    case PROTOCOL_CHANNELS_PPM:
      INTMODULE_TIMER->CCR1 = GET_MODULE_PPM_DELAY(INTERNAL_MODULE) * 2;
      INTMODULE_TIMER->CCER = INTMODULE_TIMER_OUTPUT_ENABLE | (GET_MODULE_PPM_POLARITY(INTERNAL_MODULE) ? INTMODULE_TIMER_OUTPUT_POLARITY : 0); //     // we are using complementary output so logic has to be reversed here
      INTMODULE_TIMER->CCR2 = *(intmodulePulsesData.ppm.ptr - 1) - 4000; // 2mS in advance
      INTMODULE_DMA_STREAM->CR &= ~DMA_SxCR_EN; // Disable DMA
      INTMODULE_DMA_STREAM->CR |= INTMODULE_DMA_CHANNEL | DMA_SxCR_DIR_0 | DMA_SxCR_MINC | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 | DMA_SxCR_PL_0 | DMA_SxCR_PL_1;
      INTMODULE_DMA_STREAM->PAR = CONVERT_PTR_UINT(&INTMODULE_TIMER->ARR);
      INTMODULE_DMA_STREAM->M0AR = CONVERT_PTR_UINT(intmodulePulsesData.ppm.pulses);
      INTMODULE_DMA_STREAM->NDTR = intmodulePulsesData.ppm.ptr - intmodulePulsesData.ppm.pulses;
      INTMODULE_DMA_STREAM->CR |= DMA_SxCR_EN | DMA_SxCR_TCIE; // Enable DMA
      break;
#endif

#if defined(PXX1)
    case PROTOCOL_CHANNELS_PXX1_PULSES:
      INTMODULE_TIMER->CCR2 = intmodulePulsesData.pxx.getLast() - 4000; // 2mS in advance
      INTMODULE_DMA_STREAM->CR &= ~DMA_SxCR_EN; // Disable DMA
      INTMODULE_DMA_STREAM->CR |= INTMODULE_DMA_CHANNEL | DMA_SxCR_DIR_0 | DMA_SxCR_MINC | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 | DMA_SxCR_PL_0 | DMA_SxCR_PL_1;
      INTMODULE_DMA_STREAM->PAR = CONVERT_PTR_UINT(&INTMODULE_TIMER->ARR);
      INTMODULE_DMA_STREAM->M0AR = CONVERT_PTR_UINT(intmodulePulsesData.pxx.getData());
      INTMODULE_DMA_STREAM->NDTR = intmodulePulsesData.pxx.getSize();
      INTMODULE_DMA_STREAM->CR |= DMA_SxCR_EN | DMA_SxCR_TCIE; // Enable DMA
      break;
#endif

#if defined(PXX1) && defined(HARDWARE_INTERNAL_MODULE_SIZE_SML)
    case PROTOCOL_CHANNELS_PXX1_SERIAL:
      intmoduleSendBuffer(intmodulePulsesData.pxx_uart.getData(), intmodulePulsesData.pxx_uart.getSize());
      break;
#endif

#if defined(PXX2)
    case PROTOCOL_CHANNELS_PXX2_HIGHSPEED:
    case PROTOCOL_CHANNELS_PXX2_LOWSPEED:
      intmoduleSendBuffer(intmodulePulsesData.pxx2.getData(), intmodulePulsesData.pxx2.getSize());
      break;
#endif

#if defined(SBUS) || defined(DSM2) || defined(MULTIMODULE)
    case PROTOCOL_CHANNELS_SBUS:
      INTMODULE_TIMER->CCER = INTMODULE_TIMER_OUTPUT_ENABLE | (GET_SBUS_POLARITY(INTERNAL_MODULE) ? 0 : INTMODULE_TIMER_OUTPUT_POLARITY); // reverse polarity for Sbus if needed
      // no break
    case PROTOCOL_CHANNELS_DSM2_LP45:
    case PROTOCOL_CHANNELS_DSM2_DSM2:
    case PROTOCOL_CHANNELS_DSM2_DSMX:
    case PROTOCOL_CHANNELS_MULTIMODULE:
      INTMODULE_TIMER->CCR2 = intmodulePulsesData.multi.getLast() - 4000; // 2mS in advance
      INTMODULE_DMA_STREAM->CR &= ~DMA_SxCR_EN; // Disable DMA
      INTMODULE_DMA_STREAM->CR |= INTMODULE_DMA_CHANNEL | DMA_SxCR_DIR_0 | DMA_SxCR_MINC | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 | DMA_SxCR_PL_0 | DMA_SxCR_PL_1;
      INTMODULE_DMA_STREAM->PAR = CONVERT_PTR_UINT(&INTMODULE_TIMER->ARR);
      INTMODULE_DMA_STREAM->M0AR = CONVERT_PTR_UINT(intmodulePulsesData.multi.getData());
      INTMODULE_DMA_STREAM->NDTR = intmodulePulsesData.multi.getSize();
      INTMODULE_DMA_STREAM->CR |= DMA_SxCR_EN | DMA_SxCR_TCIE; // Enable DMA
      break;
#endif

    default:
      INTMODULE_TIMER->DIER |= TIM_DIER_CC2IE;
      break;
  }
}

extern "C" void INTMODULE_DMA_STREAM_IRQHandler()
{
  if (!DMA_GetITStatus(INTMODULE_DMA_STREAM, INTMODULE_DMA_FLAG_TC))
    return;

  DMA_ClearITPendingBit(INTMODULE_DMA_STREAM, INTMODULE_DMA_FLAG_TC);

  INTMODULE_TIMER->SR &= ~TIM_SR_CC2IF; // Clear flag
  INTMODULE_TIMER->DIER |= TIM_DIER_CC2IE; // Enable this interrupt
}

extern "C" void INTMODULE_TIMER_CC_IRQHandler()
{
  INTMODULE_TIMER->DIER &= ~TIM_DIER_CC2IE; // Stop this interrupt
  INTMODULE_TIMER->SR &= ~TIM_SR_CC2IF;
  if (setupPulsesExternalModule()) {
    intmoduleSendNextFrame();
  }
}
