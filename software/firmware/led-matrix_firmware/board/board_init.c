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

#include "board_init.h"
#include "hal_gpio.h"

/*
* Definitions.
*/

/*
* Declerations.
*/
void BOARD_InitDebugConsole(void);

/*
* Functions.
*/
void BOARD_Init(void)
{
    BOARD_InitBootClocks();
    BOARD_InitPins();

    BOARD_InitDebugConsole();
}

void BOARD_InitDebugConsole(void)
{
    /* Initialization USART asynchronous mode. */
    USART_InitAsync_Type usart_init;
    usart_init.EnableSwapTxRxXferSignal = false;
    usart_init.Parity = USART_Parity_None;
    usart_init.StopBits = USART_StopBits_1;
    usart_init.WordLength = USART_WordLength_8b;
    usart_init.XferMode = USART_XferMode_RxTx;
    usart_init.XferSignal = USART_XferSignal_Normal;

    /* Set USART asynchronous baudrate. */
    USART_AsyncBaudrate_Type usart_baudrate;
    usart_baudrate.BaudRate = BOARD_DEBUG_USART_BAUDRATE;
    usart_baudrate.ClockFreqHz = BOARD_DEBUG_USART_FREQ;
    usart_baudrate.OverSampling = USART_OverSampling_16;

    USART_InitAsync(BOARD_DEBUG_USART_PORT, &usart_init);
    USART_SetBaudrateAsync(BOARD_DEBUG_USART_PORT, &usart_baudrate);
    USART_Enable(BOARD_DEBUG_USART_PORT, true);  /* Enable USART. */
}

#if defined(__ARMCC_VERSION)
int fputc(int c, FILE *f)
{
    (void)(f);
    while ( 0u == ( USART_STATUS_TX_EMPTY & USART_GetStatus(BOARD_DEBUG_USART_PORT) ) )
    {
    }
    USART_PutData(BOARD_DEBUG_USART_PORT, (uint8_t)(c) );
    return c;
}

int fgetc(FILE *f)
{
    (void)(f);
    while ( 0u == (USART_STATUS_RX_NOTEMPTY & USART_GetStatus(BOARD_DEBUG_USART_PORT) ) )
    {
    }
    return USART_GetData(BOARD_DEBUG_USART_PORT);
}

#elif defined(__GNUC__)

/*
 * Called by libc stdio fwrite functions
 */
int _write(int fd, char *ptr, int len)
{
    int i = 0;

    /*
     * write "len" of char from "ptr" to file id "fd"
     * Return number of char written.
     *
    * Only work for STDOUT, STDIN, and STDERR
     */
    if (fd > 2)
    {
        return -1;
    }

    while (*ptr && (i < len))
    {
        while ( 0u == (USART_STATUS_TX_EMPTY & USART_GetStatus(BOARD_DEBUG_USART_PORT)) )
        {}
        USART_PutData(BOARD_DEBUG_USART_PORT, (uint8_t)(*ptr));
        i++;
        ptr++;
    }

    return i;
}

/*
 * Called by the libc stdio fread fucntions
 *
 * Implements a buffered read with line editing.
 */
int _read(int fd, char *ptr, int len)
{
    int my_len;

    if (fd > 2)
    {
        return -1;
    }

    my_len = 0;
    while (len > 0)
    {
        while ( 0u == (USART_STATUS_RX_NOTEMPTY & USART_GetStatus(BOARD_DEBUG_USART_PORT)) )
        {}
        *ptr = USART_GetData(BOARD_DEBUG_USART_PORT);
        len--;
        my_len++;

        if ( (*ptr == '\r') || (*ptr == '\n') || (*ptr == '\0') )
        {
            break;
        }

        ptr++;
    }

    return my_len; /* return the length we got */
}


int putchar(int c)
{
    while ( 0u == (USART_STATUS_TX_EMPTY & USART_GetStatus(BOARD_DEBUG_USART_PORT)) )
    {}
    USART_PutData(BOARD_DEBUG_USART_PORT, (uint8_t)(c));
    return c;
}

int getchar(void)
{
    while ( 0u == (USART_STATUS_RX_NOTEMPTY & USART_GetStatus(BOARD_DEBUG_USART_PORT)) )
    {}
    return USART_GetData(BOARD_DEBUG_USART_PORT);
}

#elif (defined(__ICCARM__))

/*
 * Called by libc stdio fwrite functions
 */
int __write(int fd, char *ptr, int len)
{
    int i = 0;

    /*
     * write "len" of char from "ptr" to file id "fd"
     * Return number of char written.
     *
    * Only work for STDOUT, STDIN, and STDERR
     */
    if (fd > 2)
    {
        return -1;
    }

    while (*ptr && (i < len))
    {
        while ( 0u == (USART_STATUS_TX_EMPTY & USART_GetStatus(BOARD_DEBUG_USART_PORT)) )
        {}
        USART_PutData(BOARD_DEBUG_USART_PORT, (uint8_t)(*ptr));
        i++;
        ptr++;
    }

    return i;
}

/*
 * Called by the libc stdio fread fucntions
 *
 * Implements a buffered read with line editing.
 */
int __read(int fd, char *ptr, int len)
{
    int my_len;

    if (fd > 2)
    {
        return -1;
    }

    my_len = 0;
    while (len > 0)
    {
        while ( 0u == (USART_STATUS_RX_NOTEMPTY & USART_GetStatus(BOARD_DEBUG_USART_PORT)) )
        {}
        *ptr = USART_GetData(BOARD_DEBUG_USART_PORT);
        len--;
        my_len++;

        if ( (*ptr == '\r') || (*ptr == '\n') || (*ptr == '\0') )
        {
            break;
        }

        ptr++;
    }

    return my_len; /* return the length we got */
}

int fputc(int ch, FILE *f)
{
    while ( 0u == (USART_STATUS_TX_EMPTY & USART_GetStatus(BOARD_DEBUG_USART_PORT)) )
    {}
    USART_PutData(BOARD_DEBUG_USART_PORT, (uint8_t)(ch));
    return ch;
}

int fgetc(FILE *f)
{
    while ( 0u == (USART_STATUS_RX_NOTEMPTY & USART_GetStatus(BOARD_DEBUG_USART_PORT)) )
    {}
    return USART_GetData(BOARD_DEBUG_USART_PORT);
}

#endif
