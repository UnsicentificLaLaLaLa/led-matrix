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


uint8_t gram[8] = {0};
volatile uint64_t systime = 0;

struct x_ctrl_if {
    GPIO_Type * port;
    uint32_t    pin;
};

const struct x_ctrl_if x_ctrl_list[8] = {
    {GPIOA, GPIO_PIN_0},
    {GPIOA, GPIO_PIN_1},
    {GPIOB, GPIO_PIN_1},
    {GPIOB, GPIO_PIN_0},
    {GPIOA, GPIO_PIN_15},
    {GPIOA, GPIO_PIN_2},
    {GPIOA, GPIO_PIN_14},
    {GPIOA, GPIO_PIN_13},
};

uint32_t x_ctrl_index = 0;

void led_matrix_init(void);
bool getdata_timeout(uint8_t *val, uint32_t timeout_ms);
void putdata(uint8_t val);

int main(void)
{
    uint8_t ch;

    BOARD_Init();

    printf("hello, world\r\n");
    led_matrix_init();

    uint8_t rx_val = 0;
    while(getdata_timeout(&rx_val, 2))
    {
    }

    uint32_t gram_buf_index = 0;
    while (1)
    {
        if (getdata_timeout(&rx_val, 2))
        {
            if (gram_buf_index == 8)
            {
                putdata(rx_val);
            }
            else
            {
                gram[gram_buf_index] = rx_val;
                gram_buf_index++;
            }
        }
        else
        {
            gram_buf_index = 0;
        }
    }
}

void led_matrix_init(void)
{
    memset(gram, 0, sizeof(gram));
    SysTick_Config(CLOCK_SYS_FREQ / 2000);
}

bool getdata_timeout(uint8_t *val, uint32_t timeout_ms)
{
    uint32_t time_start = systime;
    uint32_t timeout    = time_start + (timeout_ms * 2);

    while(systime < timeout && 0 == (BOARD_DEBUG_USART_PORT->SR & USART_SR_RXNE_MASK))
    {
    }

    if (0 != (BOARD_DEBUG_USART_PORT->SR & USART_SR_RXNE_MASK))
    {
        *val = BOARD_DEBUG_USART_PORT->DR;
        return true;
    }
    else
    {
        return false;
    }
}

void putdata(uint8_t val)
{
    while (0 == (BOARD_DEBUG_USART_PORT->SR & USART_SR_TXE_MASK)) {}
    BOARD_DEBUG_USART_PORT->DR = val;
}

void SysTick_Handler(void)
{
    x_ctrl_list[x_ctrl_index].port->BSRR = x_ctrl_list[x_ctrl_index].pin;
    x_ctrl_index++;
    x_ctrl_index %= 8;

    GPIOA->BRR  = 0xFF << 4;
    GPIOA->BSRR = gram[x_ctrl_index] << 4;

    x_ctrl_list[x_ctrl_index].port->BRR = x_ctrl_list[x_ctrl_index].pin;
    systime++;
}
