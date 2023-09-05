/*
 * Copyright 2023 MindMotion Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hal_adc.h"

void ADC_Init(ADC_Type * ADCx, ADC_Init_Type * init)
{
    uint32_t cfg;

    if (!init)
    {
        return;
    }
    cfg = ADCx->ADCFG & ~(    ADC_ADCFG_ADCPREH_MASK
                            |  ADC_ADCFG_ADCPREL_MASK
                            |  ADC_ADCFG_RSLTCTL_MASK
                            |  ADC_ADCR_ALIGN_MASK )
            ;
    /* Prescaler & Resolution. */
    cfg |= ADC_ADCFG_ADCPREL(init->ClockDiv)
         | ADC_ADCFG_ADCPREH((init->ClockDiv)>>1)
         | ADC_ADCFG_RSLTCTL(init->Resolution)
         ;
    ADCx->ADCFG = cfg;
    /* ADC conversion mode and conversion data result align. */
    ADCx->ADCR = (ADCx->ADCR & ~( ADC_ADCR_ADMD_MASK | ADC_ADCR_ALIGN_MASK) )
               | ADC_ADCR_ADMD(init->ConvMode)
               | ADC_ADCR_ALIGN(init->Align)
               ;
}

void ADC_Enable(ADC_Type * ADCx, bool enable)
{
    if (enable)
    {
        ADCx->ADCFG |= ADC_ADCFG_ADEN_MASK;
    }
    else
    {
        ADCx->ADCFG &= ~ADC_ADCFG_ADEN_MASK;
    }
}

/* Use VBG 1.2V as default voltage sensor. */
void  ADC_EnableVoltSensor(ADC_Type * ADCx, bool enable)
{
    if (enable)
    {
        ADCx ->ADCFG |= ADC_ADCFG_VSEN_MASK;
    }
    else
    {
        ADCx->ADCFG &= ~ADC_ADCFG_VSEN_MASK;
    }
}

void ADC_EnableInterrupts(ADC_Type * ADCx, uint32_t interrupts, bool enable)
{
    if (enable)
    {
        if ( 0u != (ADC_INT_CONV_SEQ_DONE & interrupts) )
        {
            ADCx->ADCR |= ADC_ADCR_EOSIE_MASK;
        }
        if ( 0u != (ADC_INT_CONV_COMPARE_DONE & interrupts) )
        {
            ADCx->ADCR |= ADC_ADCR_AWDIE_MASK;
        }

    }
    else
    {
        if ( 0u != (ADC_INT_CONV_SEQ_DONE & interrupts) )
        {
            ADCx->ADCR &= ~ADC_ADCR_EOSIE_MASK;
        }
        if ( 0u != (ADC_INT_CONV_COMPARE_DONE & interrupts) )
        {
            ADCx->ADCR &= ~ADC_ADCR_AWDIE_MASK;
        }
    }
}

uint32_t ADC_GetStatus(ADC_Type * ADCx)
{
    uint32_t flags = 0u;

    if ( 0u != (ADC_ADSTA_EOSIF_MASK & ADCx->ADSTA) )
    {
        flags |= ADC_STATUS_CONV_SEQ_DONE;
    }
    if ( 0u != (ADC_ADSTA_AWDIF_MASK & ADCx->ADSTA) )
    {
        flags |= ADC_STATUS_CONV_COMPARE_DONE;
    }

     return flags;
}

void ADC_ClearStatus(ADC_Type * ADCx, uint32_t flags)
{

    if ( 0u != (ADC_STATUS_CONV_SEQ_DONE & flags) )
    {
        ADCx->ADSTA = ADC_ADSTA_EOSIF_MASK;
    }
    if ( 0u != (ADC_STATUS_CONV_COMPARE_DONE & flags) )
    {
        ADCx->ADSTA = ADC_ADSTA_AWDIF_MASK;
    }
}

uint32_t ADC_GetConvResult(ADC_Type * ADCx, uint32_t * channel, uint32_t * flags)
{
    uint32_t tmp32 = ADCx->ADDATA;

    *channel = (tmp32 & ADC_ADDATA_CHANNELSEL_MASK ) >> ADC_ADDATA_CHANNELSEL_SHIFT;
    if (flags)
    {
        *flags = (tmp32 & (ADC_ADDATA_OVERRUN_MASK | ADC_ADDATA_VALID_MASK) ) >> ADC_ADDATA_OVERRUN_SHIFT;
    }

    return (tmp32 & ADC_ADDATA_DATA_MASK ) >> ADC_ADDATA_DATA_SHIFT;
}

uint32_t ADC_GetSlotConvResult(ADC_Type * ADCx, uint32_t slot, uint32_t * flags)
{
    uint32_t tmp32 = ADCx ->ADDR[slot];

    if (flags)
    {
        *flags = (tmp32 & (ADC_ADDR_OVERRUN_MASK | ADC_ADDR_VALID_MASK) ) >> ADC_ADDR_OVERRUN_SHIFT;
    }

    return (tmp32 & ADC_ADDR_DATA_MASK ) >> ADC_ADDR_DATA_SHIFT;
}

void  ADC_SetChnSampleTime(ADC_Type * ADCx, ADC_SampleTime_Type sample_time)
{
    ADCx->ADCFG |= sample_time;
}

void ADC_EnableHwTrigger(ADC_Type * ADCx, ADC_HwTriggerConf_Type * conf)
{
    if ( !conf )
    {
        ADCx->ADCR &= ~ADC_ADCR_TRGEN_MASK;
        return;
    }
    /* Enable the hardware trigger. */
    ADCx->ADCR = ( ADCx->ADCR & ~( ADC_ADCR_TRGSHIFT_MASK | ADC_ADCR_TRGEDGE_MASK | ADC_ADCR_TRGSELL_MASK | ADC_ADCR_TRGSELH_MASK ))
               | ADC_ADCR_TRGEN_MASK
               | ADC_ADCR_TRGSHIFT(conf->DelayCycle)
               | ADC_ADCR_TRGEDGE(conf->Edge)
               | ADC_ADCR_TRGSELL(conf->Source & 0x7)
               | ADC_ADCR_TRGSELH(conf->Source >> 3u)
               ;
}

void ADC_EnableSeqSlot(ADC_Type * ADCx, uint32_t slot_idx, uint32_t channel)
{

    if (slot_idx > 16u)
    {
        return; /* the available range of seq length is within 16u. */
    }

    /* enable any channel sequence mode. */
    ADCx->ANYCR |= ADC_ANYCR_CHANYMDEN_MASK;
    /* select the any slots number. */
    ADCx->ANYCFG = ADC_ANYCFG_CHANYNUM(slot_idx);
    uint32_t offset = slot_idx;
    /* fill the channel into each slot. */
    if (slot_idx < 8u)
    {
        ADCx->CHANY0 = (ADCx->CHANY0 & ~( 0xFF << (offset * 4u) )) | ( (channel & 0xF) << (offset * 4u) );
    }
    else
    {
        ADCx->CHANY1 = (ADCx->CHANY1 & ~( 0xFF << ((offset - 8u) * 4u) )) | ( (channel & 0xF) << ((offset - 8u) * 4u) );
    }
}

void ADC_DoSwTrigger(ADC_Type * ADCx, bool enable)
{
    if (enable)
    {
        ADCx->ADCR |= ADC_ADCR_ADST_MASK;
    }
    else
    {
        ADCx->ADCR &= ~ADC_ADCR_ADST_MASK;
    }
}

void ADC_EnableHwComp(ADC_Type * ADCx, ADC_HwCompConf_Type * conf)
{
    if ( !conf )
    {
        /* disable the hardware compare feature for both regular & any seq. */
        ADCx->ADCFG &= ~ADC_ADCFG_AWDEN_MASK;
        return;
    }

    /* enable the hardware compare feature. */
    ADCx->ADCFG |= ADC_ADCFG_AWDEN_MASK;

    /* setup the compare channel. */

    ADCx->ADCR = (ADCx->ADCR & ~(ADC_ADCR_CMPCH_MASK ) )
                                 | ADC_ADCR_CMPCH(conf->ChnNum)
                                 ;
    /* setup the compare boundary. */
    ADCx->ADCMPR = ADC_ADCMPR_CMPLDATA(conf->LowLimit)
                 | ADC_ADCMPR_CMPHDATA(conf->HighLimit)
                 ;
}

void ADC_ResetAllSlot(ADC_Type * ADCx)
{
    /* wait for the latest conversion done. */
    while( (0u != (ADCx->ADSTA & ADC_ADSTA_BUSY_MASK)))
    {}
    /* disable the any sequence. */
    ADCx->CHANY0 = 0x00;
    ADCx->CHANY1 = 0x00;
    /* disable sequence conversion. */
    ADCx->ANYCR &= ~ADC_ANYCR_CHANYMDEN_MASK;
}

/* EOF. */
