;/* ------------------------------------------------------------------------- */
;/*  @file:    startup_MM32G0001.s                                            */
;/*  @purpose: CMSIS Cortex-M0 Core Device Startup File                       */
;/*                                                                           */
;/*  @version: 1.0                                                            */
;/*  @date:    2023-04-27                                                     */
;/*  @build:   b230427                                                        */
;/* ------------------------------------------------------------------------- */
;/*                                                                           */
;/* Copyright 2023 MindMotion                                                 */
;/* All rights reserved.                                                      */
;/*                                                                           */
;/* SPDX-License-Identifier: BSD-3-Clause                                     */
;/*****************************************************************************/
;/* Version:  MDK for ARM Embedded Processors                                  */
;/*****************************************************************************/
                PRESERVE8
                THUMB

; Vector Table Mapped to Address 0 at Reset
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size
                IMPORT  |Image$$ARM_LIB_STACK$$ZI$$Limit|

__Vectors       DCD     |Image$$ARM_LIB_STACK$$ZI$$Limit|                       ;         Top of Stack
                DCD     Reset_Handler                                           ;  -15    Reset
                DCD     NMI_Handler                                             ;  -14    NMI
                DCD     Hardwarefault_Handler                                   ;  -13    Hardwarefault
                DCD     0                                                       ;  -12    Reserved
                DCD     0                                                       ;  -11    Reserved
                DCD     0                                                       ;  -10    Reserved
                DCD     0                                                       ;  -9    Reserved
                DCD     0                                                       ;  -8    Reserved
                DCD     0                                                       ;  -7    Reserved
                DCD     0                                                       ;  -6    Reserved
                DCD     SVCall_Handler                                          ;  -5    SVCall
                DCD     DebugMonitor_Handler                                    ;  -4    DebugMonitor_Handler
                DCD     0                                                       ;  -3    Reserved
                DCD     PendSV_Handler                                          ;  -2    PendSV
                DCD     SysTick_Handler                                         ;  -1    SysTick
                DCD     IWDG_IRQHandler                                         ;  0    IWDG
                DCD     PVD_VDT_IRQHandler                                      ;  1    PVD_VDT
                DCD     0                                                       ;  2    Reserved
                DCD     FLASH_IRQHandler                                        ;  3    FLASH
                DCD     RCC_IRQHandler                                          ;  4    RCC
                DCD     EXTI0_1_IRQHandler                                      ;  5    EXTI0_1
                DCD     EXTI2_3_IRQHandler                                      ;  6    EXTI2_3
                DCD     EXTI4_15_IRQHandler                                     ;  7    EXTI4_15
                DCD     0                                                       ;  8    Reserved
                DCD     0                                                       ;  9    Reserved
                DCD     0                                                       ;  10    Reserved
                DCD     0                                                       ;  11    Reserved
                DCD     ADC1_IRQHandler                                         ;  12    ADC1
                DCD     TIM1_BRK_UP_TRG_COM_IRQHandler                          ;  13    TIM1_BRK_UP_TRG_COM
                DCD     TIM1_CC_IRQHandler                                      ;  14    TIM1_CC
                DCD     0                                                       ;  15    Reserved
                DCD     TIM3_IRQHandler                                         ;  16    TIM3
                DCD     0                                                       ;  17    Reserved
                DCD     0                                                       ;  18    Reserved
                DCD     TIM14_IRQHandler                                        ;  19    TIM14
                DCD     0                                                       ;  20    Reserved
                DCD     0                                                       ;  21    Reserved
                DCD     0                                                       ;  22    Reserved
                DCD     I2C1_IRQHandler                                         ;  23    I2C1
                DCD     0                                                       ;  24    Reserved
                DCD     SPI1_IRQHandler                                         ;  25    SPI1
                DCD     0                                                       ;  26    Reserved
                DCD     USART1_IRQHandler                                       ;  27    USART1
                DCD     USART2_IRQHandler                                       ;  28    USART2



__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY

; Reset handler
Reset_Handler   PROC
                EXPORT  Reset_Handler              [WEAK]
                IMPORT  SystemInit
                IMPORT  __main

                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP

; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler                [WEAK]
                B       .
                ENDP
Hardwarefault_Handler\
                PROC
                EXPORT  Hardwarefault_Handler      [WEAK]
                B       .
                ENDP
SVCall_Handler  PROC
                EXPORT  SVCall_Handler             [WEAK]
                B       .
                ENDP
DebugMonitor_Handler\
                PROC
                EXPORT  DebugMonitor_Handler       [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler             [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler            [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  IWDG_IRQHandler                [WEAK]
                EXPORT  PVD_VDT_IRQHandler              [WEAK]
                EXPORT  FLASH_IRQHandler                [WEAK]
                EXPORT  RCC_IRQHandler                  [WEAK]
                EXPORT  EXTI0_1_IRQHandler              [WEAK]
                EXPORT  EXTI2_3_IRQHandler              [WEAK]
                EXPORT  EXTI4_15_IRQHandler             [WEAK]
                EXPORT  ADC1_IRQHandler                 [WEAK]
                EXPORT  TIM1_BRK_UP_TRG_COM_IRQHandler  [WEAK]
                EXPORT  TIM1_CC_IRQHandler              [WEAK]
                EXPORT  TIM3_IRQHandler                 [WEAK]
                EXPORT  TIM14_IRQHandler                [WEAK]
                EXPORT  I2C1_IRQHandler                 [WEAK]
                EXPORT  SPI1_IRQHandler                 [WEAK]
                EXPORT  USART1_IRQHandler               [WEAK]
                EXPORT  USART2_IRQHandler               [WEAK]

IWDG_IRQHandler
PVD_VDT_IRQHandler
FLASH_IRQHandler
RCC_IRQHandler
EXTI0_1_IRQHandler
EXTI2_3_IRQHandler
EXTI4_15_IRQHandler
ADC1_IRQHandler
TIM1_BRK_UP_TRG_COM_IRQHandler
TIM1_CC_IRQHandler
TIM3_IRQHandler
TIM14_IRQHandler
I2C1_IRQHandler
SPI1_IRQHandler
USART1_IRQHandler
USART2_IRQHandler

                B       .
                ENDP
                ALIGN
                END