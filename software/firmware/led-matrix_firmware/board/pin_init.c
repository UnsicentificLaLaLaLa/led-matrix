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

#include "clock_init.h"
#include "pin_init.h"
#include "hal_rcc.h"
#include "hal_gpio.h"

void BOARD_InitPins(void)
{
    GPIO_Init_Type gpio_init;

    gpio_init.Pins  = GPIO_PIN_3;
    gpio_init.PinMode  = GPIO_PinMode_In_PullUp; 
    gpio_init.Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio_init);
    GPIO_PinAFConf(GPIOA, gpio_init.Pins, GPIO_AF_15);

    if (GPIO_ReadInDataBit(GPIOA, gpio_init.Pins))
    {
        GPIO_PinAFConf(GPIOA, gpio_init.Pins, GPIO_AF_1);

        gpio_init.Pins  = GPIO_PIN_13 | GPIO_PIN_14;
        gpio_init.PinMode  = GPIO_PinMode_Out_OpenDrain; 
        gpio_init.Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &gpio_init);
        GPIO_SetBits(GPIOA, gpio_init.Pins);
        GPIO_PinAFConf(GPIOA, gpio_init.Pins, GPIO_AF_15);
    }
    else 
    {
        gpio_init.Pins  = GPIO_PIN_12;
        gpio_init.PinMode  = GPIO_PinMode_Out_PushPull; 
        gpio_init.Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &gpio_init);
        GPIO_ClearBits(GPIOA, gpio_init.Pins);
        GPIO_PinAFConf(GPIOA, gpio_init.Pins, GPIO_AF_15);
        while (1) {}
    }

    gpio_init.Pins  = GPIO_PIN_12;
    gpio_init.PinMode  = GPIO_PinMode_AF_PushPull; 
    gpio_init.Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio_init);
    GPIO_PinAFConf(GPIOA, gpio_init.Pins, GPIO_AF_1);

    gpio_init.Pins  = 0xFF << 4;
    gpio_init.PinMode  = GPIO_PinMode_Out_PushPull; 
    gpio_init.Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio_init);
    GPIO_ClearBits(GPIOA, gpio_init.Pins);
    GPIO_PinAFConf(GPIOA, gpio_init.Pins, GPIO_AF_15);

    gpio_init.Pins  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_15 | GPIO_PIN_2;
    gpio_init.PinMode  = GPIO_PinMode_Out_OpenDrain; 
    gpio_init.Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio_init);
    GPIO_SetBits(GPIOA, gpio_init.Pins);
    GPIO_PinAFConf(GPIOA, gpio_init.Pins, GPIO_AF_15);

    gpio_init.Pins  = GPIO_PIN_0 | GPIO_PIN_1;
    gpio_init.PinMode  = GPIO_PinMode_Out_OpenDrain; 
    gpio_init.Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio_init);
    GPIO_SetBits(GPIOB, gpio_init.Pins);
    GPIO_PinAFConf(GPIOB, gpio_init.Pins, GPIO_AF_15);

}
