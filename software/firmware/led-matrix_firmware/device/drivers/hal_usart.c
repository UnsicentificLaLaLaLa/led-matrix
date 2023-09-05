/*
 * Copyright 2021 MindMotion Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hal_usart.h"

/* Initialization sync usart. */
void USART_InitSync(USART_Type * USARTx, USART_InitSync_Type * init)
{
    uint32_t cr1 = USARTx->CR1 & ~(   USART_CR1_SAS_MASK | USART_CR1_MLS_MASK | USART_CR1_OVER8_MASK
                                    | USART_CR1_UE_MASK | USART_CR1_DL_MASK | USART_CR1_PCE_MASK
                                    | USART_CR1_TE_MASK | USART_CR1_RE_MASK );

    /* Select synchronous mode, set xfer sequence and word length. */
    cr1 |= USART_CR1_SAS_MASK | USART_CR1_MLS(init->XferSequence) | USART_CR1_DL(init->WordLength) | USARTx->CR1 | USART_CR1_OVER8(init->OverSampling);

    /* Parity. */
    if (init->Parity == USART_Parity_Even)
    {
        cr1 |= USART_CR1_PCE_MASK;
    }
    if (init->Parity == USART_Parity_Odd)
    {
        cr1 |= USART_CR1_PCE_MASK | USART_CR1_PS_MASK;
    }
    USARTx->CR1 = cr1;

    uint32_t cr2 = USARTx->CR2 & ~( USART_CR2_SWAP_MASK | USART_CR2_STOP_MASK | USART_CR2_CPOL_MASK | USART_CR2_CPHA_MASK);

    /* SwapTxRxXferSignal, StopBits, PolPha, USART ClockOutput. */
    cr2 |= (  USART_CR2_SWAP(init->EnableSwapTxRxXferSignal) | USART_CR2_STOP(init->StopBits) 
           | ( (USART_CR2_CPHA_MASK | USART_CR2_CPOL_MASK) & ( (init->PolPha) << USART_CR2_CPHA_SHIFT ) ) );
    USARTx->CR2 = cr2;

    uint32_t cr3 = USARTx->CR3 & ~( USART_CR3_TXTOG_MASK |  USART_CR3_RXTOG_MASK | USART_CR3_HDSEL_MASK );

    /* XferSignal, Clock externally input. */
    cr3 |= ( ( ( USART_CR3_TXTOG_MASK |  USART_CR3_RXTOG_MASK) & ( (uint32_t)(init->XferSignal) <<  USART_CR3_RXTOG_SHIFT ) )
               | USART_CR3_CKINE(init->EnableClockExternallyInput) );
    USARTx->CR3 = cr3;

    /* XferMode. */
    USARTx->CR1 |= ((uint32_t)(init->XferMode) << USART_CR1_RE_SHIFT);
}

/* Initialization async usart. */
void USART_InitAsync(USART_Type * USARTx, USART_InitAsync_Type * init)
{
    uint32_t cr1 = USARTx->CR1 & ~(   USART_CR1_SAS_MASK | USART_CR1_MLS_MASK | USART_CR1_OVER8_MASK
                                    | USART_CR1_UE_MASK | USART_CR1_DL_MASK | USART_CR1_PCE_MASK
                                    | USART_CR1_TE_MASK | USART_CR1_RE_MASK);

    /* XferSequence, WordLength. */
    cr1 |= USART_CR1_DL(init->WordLength) ;

    /* Parity. */
    if (init->Parity == USART_Parity_Even)
    {
        cr1 |= USART_CR1_PCE_MASK;
    }
    if (init->Parity == USART_Parity_Odd)
    {
        cr1 |= USART_CR1_PCE_MASK | USART_CR1_PS_MASK;
    }
    USARTx->CR1 = cr1;

    uint32_t cr2 = USARTx->CR2 & ~(  USART_CR2_SWAP_MASK | USART_CR2_STOP_MASK );

    /* SwapTxRxXferSignal, StopBits. */
    cr2 |= ( USART_CR2_SWAP(init->EnableSwapTxRxXferSignal) | USART_CR2_STOP(init->StopBits) );
    USARTx->CR2 = cr2;

    uint32_t cr3 = USARTx->CR3 & ~( USART_CR3_TXTOG_MASK |  USART_CR3_RXTOG_MASK | USART_CR3_HDSEL_MASK);

    /* XferSignal. */
    cr3 |= ( (USART_CR3_TXTOG_MASK |  USART_CR3_RXTOG_MASK) & ( (uint32_t)(init->XferSignal) <<  USART_CR3_RXTOG_SHIFT ) );
    USARTx->CR3 = cr3;

    /* XferMode. */
    USARTx->CR1 |= ((uint32_t)(init->XferMode) << USART_CR1_RE_SHIFT);
    /* clear data. */
//    USARTx->DR = 0u;
}

/* Set usart synchronous baudrate. */
void USART_SetBaudrateSync(USART_Type * USARTx, USART_SyncBaudrate_Type * sync)
{
    USARTx->BRR &= ~(USART_BRR_FFD_MASK | USART_BRR_MFD_MASK);
    USARTx->BRR |= USART_BRR_MFD( (sync->ClockFreqHz / sync->BaudRate) / 4u);
}

/* set usart asynchronous baudrate. */
void USART_SetBaudrateAsync(USART_Type * USARTx, USART_AsyncBaudrate_Type * async)
{
    USARTx->BRR &= ~(USART_BRR_FFD_MASK | USART_BRR_MFD_MASK);
    uint8_t N = 8u * (2u - async->OverSampling);
    USARTx->CR1 |= USART_CR1_OVER8(async->OverSampling);
    uint32_t temp = async->ClockFreqHz / async->BaudRate;
    USARTx->BRR |= USART_BRR_MFD( temp / N ) | USART_BRR_FFD( temp % N );
}

/* USART put data. */
void USART_PutData(USART_Type * USARTx, uint8_t value)
{
    USARTx->DR = value & 0x1FF;
}

/* USART get data. */
uint8_t USART_GetData(USART_Type * USARTx)
{
    return (USARTx->DR & 0x1FF);
}

/* Enable USART. */
void USART_Enable(USART_Type * USARTx, bool enable)
{
    if (enable)
    {
        USARTx->CR1 |= USART_CR1_UE_MASK;
    }
    else
    {
        USARTx->CR1 &= ~USART_CR1_UE_MASK;
    }
 
}

/* Enable USART interrupts, use USART_STATUS. */
void USART_EnableInterrupts(USART_Type * USARTx, uint32_t interrupts, bool enable)
{
    if (enable)
    {
        USARTx->CR1 |= interrupts;
    }
    else
    {
        USARTx->CR1 &= ~interrupts;
    }
}

/* Get USART status. */
uint32_t USART_GetStatus(USART_Type * USARTx)
{
    return USARTx->SR;
}

uint32_t USART_GetEnabledInterrupts(USART_Type * USARTx)
{
    return USARTx->CR1;
}
