/*!
    \file    main.c
    \brief   USART printf demo

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
#include "gd32h759i_eval.h"
#include "systick.h"
#include <stdio.h>

static void test_status_led_init(void);
static void led_flash(uint8_t times);
static void usart_config(void);
void cache_enable(void);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/

int main(void)
{
    /* enable the CPU cache */
    cache_enable();
    /* initialize the LEDs */
    test_status_led_init();

    /* configure systick */
    systick_config();

    /* flash the LEDs for 2 time */
    led_flash(2);

    /* configure USART0 */
    usart_config();

    /* configure TAMPER key */
    gd_eval_key_init(KEY_TAMPER, KEY_MODE_GPIO);

    /* output a message on hyperterminal using printf function */
    printf("\r\n USART printf example: please press the Tamper key \r\n");

    /* wait for completion of USART transmission */
    while(RESET == usart_flag_get(EVAL_COM, USART_FLAG_TC)) {
    }

    while(1) {
        /* check if the tamper key is pressed */
        if(RESET == gd_eval_key_state_get(KEY_TAMPER)) {
            delay_ms(50);

            if(RESET == gd_eval_key_state_get(KEY_TAMPER)) {
                delay_ms(50);

                if(RESET == gd_eval_key_state_get(KEY_TAMPER)) {
                    /* turn on LED1 */
                    gd_eval_led_on(LED1);
                    /* output a message on hyperterminal using printf function */
                    printf("\r\n USART printf example \r\n");

                    /* wait for completion of USART transmission */
                    while(RESET == usart_flag_get(EVAL_COM, USART_FLAG_TC)) {
                    }
                } else {
                    /* turn off LED1 */
                    gd_eval_led_off(LED1);
                }
            } else {
                /* turn off LED1 */
                gd_eval_led_off(LED1);
            }
        } else {
            /* turn off LED1 */
            gd_eval_led_off(LED1);
        }
    }
}
/*!
    \brief      test status led initialize
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void test_status_led_init(void)
{
    gd_eval_led_init(LED1);
    gd_eval_led_init(LED2);
}

/*!
    \brief      flash the LED for test
    \param[in]  times: times to flash the LEDs
    \param[out] none
    \retval     none
*/
static void led_flash(uint8_t times)
{
    uint8_t i;

    for(i = 0; i < times; i++) {
        /* delay 400 ms */
        delay_ms(400);

        /* turn on LEDs */
        gd_eval_led_on(LED1);
        gd_eval_led_on(LED2);

        /* delay 400 ms */
        delay_ms(400);
        /* turn off LEDs */
        gd_eval_led_off(LED1);
        gd_eval_led_off(LED2);
    }
}

/*!
    \brief      usart configure
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void usart_config(void)
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(EVAL_COM_GPIO_CLK);

    /* enable USART clock */
    rcu_periph_clock_enable(EVAL_COM_CLK);

    /* connect port to USART0 TX */
    gpio_af_set(EVAL_COM_GPIO_PORT, EVAL_COM_AF, EVAL_COM_TX_PIN);

    /* connect port to USART0 RX */
    gpio_af_set(EVAL_COM_GPIO_PORT, EVAL_COM_AF, EVAL_COM_RX_PIN);

    /* configure USART TX as alternate function push-pull */
    gpio_mode_set(EVAL_COM_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, EVAL_COM_TX_PIN);
    gpio_output_options_set(EVAL_COM_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, EVAL_COM_TX_PIN);

    /* configure USART RX as alternate function push-pull */
    gpio_mode_set(EVAL_COM_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, EVAL_COM_RX_PIN);
    gpio_output_options_set(EVAL_COM_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, EVAL_COM_RX_PIN);

    /* USART configure */
    usart_deinit(EVAL_COM);
    usart_word_length_set(EVAL_COM, USART_WL_8BIT);
    usart_stop_bit_set(EVAL_COM, USART_STB_1BIT);
    usart_parity_config(EVAL_COM, USART_PM_NONE);
    usart_baudrate_set(EVAL_COM, 115200U);
    usart_receive_config(EVAL_COM, USART_RECEIVE_ENABLE);
    usart_transmit_config(EVAL_COM, USART_TRANSMIT_ENABLE);
    usart_enable(EVAL_COM);
}

/*!
    \brief      toggle the led every 500ms
    \param[in]  none
    \param[out] none
    \retval     none
*/
void led_spark(void)
{
    static __IO uint32_t timingdelaylocal = 0U;

    if(timingdelaylocal) {

        if(timingdelaylocal < 500U) {
            gd_eval_led_on(LED1);
            gd_eval_led_on(LED2);
        } else {
            gd_eval_led_off(LED1);
            gd_eval_led_off(LED2);
        }

        timingdelaylocal--;
    } else {
        timingdelaylocal = 1000U;
    }
}

/*!
    \brief      enable the CPU chache
    \param[in]  none
    \param[out] none
    \retval     none
*/
void cache_enable(void)
{
    /* enable i-cache */
    SCB_EnableICache();

    /* enable d-cache */
    SCB_EnableDCache();
}

#ifdef __GNUC__
/* retarget the C library printf function to the USART, in Eclipse GCC environment */
int __io_putchar(int ch)
{
    usart_data_transmit(EVAL_COM, (uint8_t) ch );
    while(RESET == usart_flag_get(EVAL_COM, USART_FLAG_TBE));
    return ch;
}
#else
/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM, (uint8_t)ch);
    while(RESET == usart_flag_get(EVAL_COM, USART_FLAG_TBE));

    return ch;
}
#endif /* __GNUC__ */
