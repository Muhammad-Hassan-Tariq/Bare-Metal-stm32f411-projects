/**
 * @file    main.cpp
 * @author  [Muhammad Hassan Tariq/ https://github.com/Muhammad-Hassan-Tariq]
 * @brief   Bare-metal implementation of DMA
 * @details Implemented DMA based array copy which uses DMA2 for copying contents
            from source to destination of size 50. Built-in LED used as
            flag for successful conversion
            // Tabular Form
            PIN 13         Output
            DMA2_Stream0   MEM-MEM Copy
 * @date    2026-01-12
 * @note    Target: STM32F411CEU6 (Black Pill)
 */

#include "../platform/drivers/stm32f411xe.h"
#include <cstdint>

uint8_t source[50];
uint8_t destin[50];
volatile bool dma_status =
    false; // volatile ensures the compiler reads from RAM every time

void dma_init() {
  // 1. Enable DMA2 Clock (M2M only works on DMA2)
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

  // 2. Ensure Stream is disabled before configuration
  DMA2_Stream0->CR &= ~DMA_SxCR_EN;
  while (DMA2_Stream0->CR & DMA_SxCR_EN)
    ; // Wait for stream to stop

  // 3. Clear all interrupt flags for Stream 0
  DMA2->LIFCR = 0x3F;

  // 4. Configure Control Register
  // DIR[1:0] = 10 (M2M), PINC = 1, MINC = 1, TCIE = 1
  DMA2_Stream0->CR = (0b10 << DMA_SxCR_DIR_Pos) | DMA_SxCR_PINC | DMA_SxCR_MINC | DMA_SxCR_TCIE | (0b10 << DMA_SxCR_PL_Pos);

  // 5. Set Addresses (In M2M: PAR = Source, M0AR = Destination)
  DMA2_Stream0->PAR = (uintptr_t)source;
  DMA2_Stream0->M0AR = (uintptr_t)destin;

  // 6. Set number of data items
  DMA2_Stream0->NDTR = 50;

  // 7. Enable Interrupt in NVIC
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

// Correct ISR Name from Startup file (extern "C" prevents C++ name mangling)
extern "C" void DMA2_Stream0_IRQHandler(void) {
  if (DMA2->LISR & DMA_LISR_TCIF0) {
    DMA2->LIFCR = DMA_LIFCR_CTCIF0; // Clear the Transfer Complete flag
    dma_status = true;
  }
}

int main(void) {
  // Initialize GPIO PC13 (Onboard LED)
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  GPIOC->MODER &= ~(3U << (13 * 2));
  GPIOC->MODER |= (1U << (13 * 2));
  GPIOC->BSRR = (1 << 13);

  // Fill source buffer
  for (int i = 0; i < 50; i++) {
    source[i] = (uint8_t)i;
  }

  dma_init();

  // Start Transfer
  DMA2_Stream0->CR |= DMA_SxCR_EN;
  for (volatile int i = 0; i < 2000000; i++)
    ;
  while (1) {
    if (dma_status) {
      // Blink LED to signal success!
      GPIOC->ODR ^= (1U << 13);
      for (volatile int i = 0; i < 200000; i++)
        ;
    }
  }
}
