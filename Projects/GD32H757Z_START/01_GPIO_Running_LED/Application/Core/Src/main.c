/*!
    \file    main.c
    \brief   running LED

    \version 2025-02-19, V2.1.0, demo for GD32H7xx
*/

/*
    Copyright (c) 2024, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32h7xx.h"
#include "systick.h"

/*!
    \brief      enable the CPU Chache
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void cache_enable(void)
{
    /* Enable I-Cache */
    SCB_EnableICache();

    /* Enable D-Cache */
    SCB_EnableDCache();
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    /* enable the CPU Cache */
    cache_enable();
    /* configure systick */
    systick_config();

    /* enable the LED GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOE);
    /* configure LED GPIO pin */ 
    gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_4);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_4);

    gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_5);

    gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_6);

    /* enable the LED GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOC);
    /* configure LED GPIO pin */ 
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_13);

    /* reset LED GPIO pin */
    gpio_bit_reset(GPIOE, GPIO_PIN_4);
    gpio_bit_reset(GPIOE, GPIO_PIN_5);
    gpio_bit_reset(GPIOE, GPIO_PIN_6);
    gpio_bit_reset(GPIOC, GPIO_PIN_13);

    while(1){
        /* turn on LED1, turn off LED4 */
        gpio_bit_set(GPIOE, GPIO_PIN_4);
        gpio_bit_reset(GPIOC, GPIO_PIN_13);
        delay_ms(500);

        /* turn on LED2, turn off LED1 */
        gpio_bit_set(GPIOE, GPIO_PIN_5);
        gpio_bit_reset(GPIOE, GPIO_PIN_4);
        delay_ms(500);

        /* turn on LED3, turn off LED2 */
        gpio_bit_set(GPIOE, GPIO_PIN_6);
        gpio_bit_reset(GPIOE, GPIO_PIN_5);
        delay_ms(500);

        /* turn on LED4, turn off LED3 */
        gpio_bit_set(GPIOC, GPIO_PIN_13);
        gpio_bit_reset(GPIOE, GPIO_PIN_6);
        delay_ms(500);
    }
}
