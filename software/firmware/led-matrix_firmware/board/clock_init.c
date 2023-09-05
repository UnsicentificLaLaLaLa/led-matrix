/*
 * MIT License
 *
 * Copyright (c) 2023 UnsicentificLaLaLaLa
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "hal_common.h"
#include "clock_init.h"

#include "hal_rcc.h"

void CLOCK_ResetToDefault(void);
void CLOCK_BootToHSI48MHz(void);

void BOARD_InitBootClocks(void)
{
    CLOCK_ResetToDefault();
    CLOCK_BootToHSI48MHz();

    /* USART. */
    RCC_EnableAPB1Periphs(RCC_APB1_PERIPH_USART1, true);
    RCC_ResetAPB1Periphs(RCC_APB1_PERIPH_USART1);

    /* GPIOA. */
    RCC_EnableAHB1Periphs(RCC_AHB1_PERIPH_GPIOA, true);
    RCC_ResetAHB1Periphs(RCC_AHB1_PERIPH_GPIOA);

    /* GPIOB. */
    RCC_EnableAHB1Periphs(RCC_AHB1_PERIPH_GPIOB, true);
    RCC_ResetAHB1Periphs(RCC_AHB1_PERIPH_GPIOB);
}

/* Switch to HSI. */
void CLOCK_ResetToDefault(void)
{
    /* Reset all other clock sources. */
    RCC->CR = RCC_CR_HSION_MASK;

    /* Disable all interrupts and clear pending bits. */
    RCC->CIR = RCC->CIR; /* clear flags. */
    RCC->CIR = 0u; /* disable interrupts. */
}

/* Select HSI 48MHz as system clock. */
void CLOCK_BootToHSI48MHz(void)
{
    RCC->APB1ENR |= (1u << 28u); /* enable PWR. */
    RCC->CR |= RCC_CR_HSION_MASK; /* Make sure the HSI is enabled. */
    while ( RCC_CR_HSIRDY_MASK != (RCC->CR & RCC_CR_HSIRDY_MASK) )
    {
    }


    /* Enable the FLASH prefetch. */
    RCC->AHBENR |= (1u << 4u); /* enable the access to FLASH. */
    FLASH->ACR = FLASH_ACR_LATENCY(1); /* setup divider. */

    /* Setup the dividers for each bus. */
    RCC->CFGR = RCC_CFGR_HPRE(0)   /* div=1 for AHB freq. */
              | RCC_CFGR_PPRE1(0)  /* div=1 for APB1 freq. */
              | RCC_CFGR_MCO(4)    /* use STSCLK as output. */
              ;

    RCC->CFGR = RCC_CFGR_SW(2u); /* Reset other clock sources and switch to HSI. */
    while ( RCC_CFGR_SWS(2u) != (RCC->CFGR & RCC_CFGR_SWS_MASK) ) /* Wait while the SYSCLK is switch to the HSI. */
    {
    }
}
