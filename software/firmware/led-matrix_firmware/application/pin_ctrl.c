/* pin_ctrl.c */

#include "board_init.h"



static uint32_t x_ctrl_index = 0;

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

void led_matrix_init(void)
{
    memset(gram, 0, sizeof(gram));
    SysTick_Config(CLOCK_SYS_FREQ / 2000);
}

bool getchar_timeout(uint8_t *val, uint32_t timeout_ms)
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