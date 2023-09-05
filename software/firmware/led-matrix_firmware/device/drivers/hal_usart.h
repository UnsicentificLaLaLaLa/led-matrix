/*
 * Copyright 2021 MindMotion Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __HAL_USART_H__
#define __HAL_USART_H__

#include "hal_common.h"

/*!
 * @addtogroup USART
 * @{
 */

/*!
 * @addtogroup USART_STATUS
 * @{
 */
#define USART_STATUS_TX_EMPTY USART_SR_TXE_MASK      /*!< Status flag when USART transmiter buffer is empty. */
#define USART_STATUS_TX_DONE  USART_SR_TC_MASK       /*!< Data is not transferred to the shift register or the last bit of the data frame was sent without writing USART_DR to update the sending data register. */
#define USART_STATUS_RX_NOTEMPTY USART_SR_RXNE_MASK  /*!< Status flag when USART receiving buffer is with available data. */
/*!
 * @}
 */

/*!
 * @brief USART byte transmission sequence of data frame type.
 */
typedef enum
{
    USART_XferSequence_Low  = 0u,   /*!< Data frame bytes are output in the order from the lowest bit to the highest bit. */
    USART_XferSequence_High = 1u,   /*!< Bytes of data frames are output in the order from the highest bit to the lowest bit. */
} USART_XferSequence_Type;

/*!
 * @brief USART stop bits type.
 */
typedef enum
{
    USART_StopBits_1   = 0u, /*!< 1 stop bits. */
    USART_StopBits_0_5 = 1u, /*!< 0.5 stop bits. */
    USART_StopBits_2   = 2u, /*!< 2 stop bits. */
    USART_StopBits_1_5 = 3u, /*!< 1.5 stop bits. */
} USART_StopBits_Type;

/*!
 * @brief USART oversampling type.
 */
typedef enum 
{
    USART_OverSampling_16 = 0u,  /*!< 16 times oversampling. */
    USART_OverSampling_8  = 1u,  /*!< 8 times oversampling. */
} USART_OverSampling_Type;

/*!
 * @brief USART xfer signal type.
 */
typedef enum
{
    USART_XferSignal_Normal = 0u,   /*!< Disable both Tx and Rx toggle. */
    USART_XferSignal_RxToggle = 1u, /*!< Enable Rx toggle only. */
    USART_XferSignal_TxToggle = 2u, /*!< Enable Tx toggle only. */
    USART_XferSignal_RxTxToggle = 3u, /*!< Enable both Tx and Rx toggle. */
} USART_XferSignal_Type;

/*!
 * @brief USART word length type.
 */
typedef enum
{
    USART_WordLength_8b = 0u,   /*!< Word length 8 bits. */
    USART_WordLength_9b = 1u,   /*!< Word length 9 bits. */
} USART_WordLength_Type;

/*!
 * @brief USART xfer mode type.
 */
typedef enum
{
    USART_XferMode_None   = 0u, /*!< Disable both Tx and Rx. */
    USART_XferMode_RxOnly = 1u, /*!< Enable Rx only. */
    USART_XferMode_TxOnly = 2u, /*!< Enable Tx only. */
    USART_XferMode_RxTx   = 3u, /*!< Enable both Rx and Tx. */
} USART_XferMode_Type;

/*!
 * @brief USART parity type.
 */
typedef enum
{
    USART_Parity_None = 0u, /*!< No parity. */
    USART_Parity_Even = 1u, /*!< Even parity. */
    USART_Parity_Odd  = 2u, /*!< Odd parity. */
} USART_Parity_Type;

/*!
 * @brief USART synchronization mode polarity phase type.
 */
typedef enum
{
    USART_PolPha_Alt0 = 0u, /*!< CPOL = 0, CPHA = 1, Clock line is low when idle, Data valid when at falling edge. */
    USART_PolPha_Alt1 = 1u, /*!< CPOL = 0, CPHA = 0, Clock line is low when idle, Data valid when at rising edge. */
    USART_PolPha_Alt2 = 2u, /*!< CPOL = 1, CPHA = 1, Clock line is high when idle, Data valid when at rising edge. */
    USART_PolPha_Alt3 = 3u, /*!< CPOL = 1, CPHA = 0, Clock line is high when idle, Data valid when at falling edge. */
} USART_PolPha_Type;

/*!
 * @brief This type of structure instance is used to keep the settings when calling the @ref USART_InitSync() to initialize the USART synchronization module.
 */
typedef struct
{
    USART_XferSequence_Type         XferSequence;        /*!< Specify byte transmission sequence of data frame. */
    USART_WordLength_Type           WordLength;          /*!< Specify the number of data bits transmitted or received in a frame. */
    USART_Parity_Type               Parity;              /*!< Specify the parity mode. */
    USART_XferMode_Type             XferMode;            /*!< Specify whether the Receive or Transmit mode is enabled or not. */
    bool                            EnableSwapTxRxXferSignal;   /*! < Input and output exchange of IO pin function. */
    USART_StopBits_Type             StopBits;            /*!< Specify the number of stop bits transmitted. */
    USART_PolPha_Type               PolPha;              /*!< Specifies different communication modes. */
    USART_XferSignal_Type           XferSignal;          /*!< Specify usart xfer signal. */
    bool                            EnableClockExternallyInput;  /*!< Specify the clock is externally input. */
    USART_OverSampling_Type         OverSampling;
} USART_InitSync_Type;

/*!
 * @brief This type of structure instance is used to keep the settings when calling the @ref USART_InitAsync() to initialize the USART asynchronous module.
 */
typedef struct
{
    USART_WordLength_Type           WordLength;          /*!< Specify the number of data bits transmitted or received in a frame. */
    USART_Parity_Type               Parity;              /*!< Specify the parity mode. */
    USART_XferMode_Type             XferMode;            /*!< Specify whether the Receive or Transmit mode is enabled or not. */
    bool                            EnableSwapTxRxXferSignal;   /*! < Input and output exchange of IO pin function. */
    USART_StopBits_Type             StopBits;            /*!< Specify the number of stop bits transmitted. */
    USART_XferSignal_Type           XferSignal;          /*!< Specify usart xfer signal. */
} USART_InitAsync_Type;

/*!
 * @brief This type of structure instance is used to keep the settings when calling the @ref USART_SetBaudrateSync() to initialize the USART synchronous baudrate module.
 */
typedef struct
{
    uint32_t                        ClockFreqHz;  /*!< Bus Clock Freq. */
    uint32_t                        BaudRate;     /*!< Specify the USART communication baudrate. */
} USART_SyncBaudrate_Type;

/*!
 * @brief This type of structure instance is used to keep the settings when calling the @ref USART_SetBaudrateAsync() to initialize the USART asynchronous baudrate module.
 */
typedef struct
{
    uint32_t                        ClockFreqHz;  /*!< Bus Clock Freq. */
    uint32_t                        BaudRate;     /*!< Specify the USART communication baud rate. */
    USART_OverSampling_Type         OverSampling; /*!< Specify oversampling is 8 times or 16 times. */
} USART_AsyncBaudrate_Type;

/*!
 * @brief Initialize the USART synchronous module.
 *
 * @param USARTx USART instance.
 * @param init Pointer to the initialization structure. See to @ref USART_InitSync_Type.
 * @return None.
 */
void USART_InitSync(USART_Type * USARTx, USART_InitSync_Type * init);

/*!
 * @brief Initialize the USART asynchronous module.
 *
 * @param USARTx USART instance.
 * @param init Pointer to the initialization structure. See to @ref USART_InitAsync_Type.
 * @return None.
 */
void USART_InitAsync(USART_Type * USARTx, USART_InitAsync_Type * init);

/*!
 * @brief Set the baudrate of USART synchronous module.
 *
 * @param USARTx USART instance.
 * @param sync Pointer to the synchronous baudrate setting structure. See to @ref USART_SyncBaudrate_Type.
 * @return None.
 */
void USART_SetBaudrateSync(USART_Type * USARTx, USART_SyncBaudrate_Type * sync);

/*!
 * @brief Set the baudrate of USART asynchronous module.
 *
 * @param USARTx USART instance.
 * @param async Pointer to the asynchronous baudrate setting structure. See to @ref USART_AsyncBaudrate_Type.
 * @return None.
 */
void USART_SetBaudrateAsync(USART_Type * USARTx, USART_AsyncBaudrate_Type * async);

/*!
 * @brief Enable the USART module.
 *
 * The USART module should be enabled before sending or receiving data.
 *
 * @param USARTx USART instance.
 * @param enable 'true' to enable the module, 'false' to disable the module.
 * @return None.
 */
void USART_Enable(USART_Type * USARTx, bool enable);

/*!
 * @brief Enable interrupts of the USART module.
 *
 * @param USARTx USART instance.
 * @param interrupts Interrupt code masks. See to @ref USART_STATUS.
 * @param enable 'true' to enable the indicated interrupts, 'false' to disable the indicated interrupts.
 * @return None.
 */
void USART_EnableInterrupts(USART_Type * USARTx, uint32_t interrupts, bool enable);

/*!
 * @brief Put the data into transmiter buffer of the USART module.
 *
 * @param USARTx USART instance.
 * @param value Data value to be send through the transmiter.
 * @return None.
 */
void USART_PutData(USART_Type * USARTx, uint8_t value);

/*!
 * @brief Get the data from receiver buffer of the USART module.
 *
 * @param USARTx USART instance.
 * @return The data value received from the receiver.
 * @return None.
 */
uint8_t USART_GetData(USART_Type * USARTx);

/*!
 * @brief Get the current status flags of the USART module.
 *
 * @param USARTx USART instance.
 * @return Status flags. See to @ref USART_STATUS.
 */
uint32_t USART_GetStatus(USART_Type * USARTx);

/*!
 * @brief Read the current enabled interrupts the USART module.
 *
 * @param USARTx USART instance.
 * @return The mask codes enabled interrupts. See to @ref USART_STATUS.
 */
uint32_t USART_GetEnabledInterrupts(USART_Type * USARTx);

/*!
 *@}
 */

#endif /* __HAL_USART_H__ */
