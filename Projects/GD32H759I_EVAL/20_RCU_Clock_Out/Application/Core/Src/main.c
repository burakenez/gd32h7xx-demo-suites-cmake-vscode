/*!
    \file    main.c
    \brief   RCU clock out demo

    \version 2024-07-31, V2.0.0, demo for GD32H7xx
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
#include <stdio.h>

typedef enum {
    CKOUT_SYSTEMCLOCK = 0,
    CKOUT_IRC64M,
    CKOUT_IRC48M,
    CKOUT_IRC32K,
    CKOUT_LXTAL,
    CKOUT_HXTAL,
    CKOUT_PLL1R,
    CKOUT_PLL2R,
    CKOUT_COUNT
} clock_output_enum;

void all_led_init(void);
void all_led_off(void);
void clock_output_config(void);
void clock_output_select(uint8_t seq);
void cache_enable(void);

extern __IO uint8_t g_button_press_flag;

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

    uint8_t func_seq = 0;
    uint8_t led_seq = 0;

    /* initialize the USART */
    gd_eval_com_init(EVAL_COM);

    printf("\r\n /=========== Gigadevice Clock Output Demo ===========/ \r\n");
    printf("press tamper key to select clock output source \r\n");

    /* initialize the LEDs */
    all_led_init();
    /* initialize the clock output */
    clock_output_config();
    /* initialize the tamper key */
    gd_eval_key_init(KEY_TAMPER, KEY_MODE_EXTI);

    while(1) {
        if(1 == g_button_press_flag) {
            /* control the led */
            all_led_off();
            gd_eval_led_on((led_typedef_enum)led_seq);
            /* select the clock output mode */
            clock_output_select(func_seq);

            func_seq++;
            func_seq %= CKOUT_COUNT;
            led_seq++;
            led_seq %= LEDn;
            g_button_press_flag = 0;
        }
    }
}

/*!
    \brief      initialize the LEDs
    \param[in]  none
    \param[out] none
    \retval     none
*/
void all_led_init(void)
{
    gd_eval_led_init(LED1);
    gd_eval_led_init(LED2);
}

/*!
    \brief      all LEDs turn off
    \param[in]  none
    \param[out] none
    \retval     none
*/
void all_led_off(void)
{
    gd_eval_led_off(LED1);
    gd_eval_led_off(LED2);
}

/*!
    \brief      configure clock output function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void clock_output_config(void)
{
    /* peripheral clock enable */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);

    /* enable IRC64M */
    rcu_osci_on(RCU_IRC64M);
    if(ERROR == rcu_osci_stab_wait(RCU_IRC64M)) {
        printf("RCU_IRC16M rcu_osci_stab_wait timeout! \r\n");
        while(1) {
        }
    }

    /* enable HXTAL */
    rcu_osci_on(RCU_HXTAL);
    if(ERROR == rcu_osci_stab_wait(RCU_HXTAL)) {
        printf("RCU_HXTAL rcu_osci_stab_wait timeout! \r\n");
        while(1) {
        }
    }

    /* enable LXTAL */
    rcu_periph_clock_enable(RCU_PMU);
    pmu_backup_write_enable();
    rcu_osci_on(RCU_LXTAL);
    if(ERROR == rcu_osci_stab_wait(RCU_LXTAL)) {
        printf("RCU_LXTAL rcu_osci_stab_wait timeout! \r\n");
        while(1) {
        }
    }
    pmu_backup_write_disable();

    /* enable IRC48M */
    rcu_osci_on(RCU_IRC48M);
    if(ERROR == rcu_osci_stab_wait(RCU_IRC48M)) {
        printf("RCU_IRC48M rcu_osci_stab_wait timeout! \r\n");
        while(1) {
        }
    }

    /* enable IRC32K */
    rcu_osci_on(RCU_IRC32K);
    if(ERROR == rcu_osci_stab_wait(RCU_IRC32K)) {
        printf("RCU_IRC32K rcu_osci_stab_wait timeout! \r\n");
        while(1) {
        }
    }

    /* enable PLL1R */
    /* configure the pll1 input and output clock range */
    rcu_pll_input_output_clock_range_config(IDX_PLL1,RCU_PLL1RNG_4M_8M,RCU_PLL1VCO_192M_836M);
    /* configure the PLL1 clock: CK_PLL1P/CK_PLL1Q/CK_PLL1R = HXTAL_VALUE / 5 * 40 / 1*/
    rcu_pll1_config(5, 40,1,1,1);
    /* enable PLL1R clock output */
    rcu_pll_clock_output_enable(RCU_PLL1R);
    rcu_osci_on(RCU_PLL1_CK);
    if(ERROR == rcu_osci_stab_wait(RCU_PLL1_CK)) {
        printf("RCU_PLL1_CK rcu_osci_stab_wait timeout! \r\n");
        while(1) {
        }
    }

    /* enable PLL2R */
    /* configure the pll2 input and output clock range */
    rcu_pll_input_output_clock_range_config(IDX_PLL2,RCU_PLL2RNG_4M_8M,RCU_PLL2VCO_192M_836M);
    /* configure the PLL2 clock: CK_PLL2P/CK_PLL2Q/CK_PLL2R = HXTAL_VALUE / 5 * (40 + 0.5) / 1 */
    rcu_pll2_config(5, 40,1,1,1);
    /* pll2 fractional latch disable */
    rcu_pll_fractional_latch_disable(IDX_PLL2);
    /* configure fractional part of the multiplication factor for PLL2 VCO */
    rcu_pll_fractional_config(IDX_PLL2, 4096);
    /* pll2 fractional latch enable */
    rcu_pll_fractional_latch_enable(IDX_PLL2);
    /* enable PLL2R clock output */
    rcu_pll_clock_output_enable(RCU_PLL2R);
    rcu_osci_on(RCU_PLL2_CK);
    if(ERROR == rcu_osci_stab_wait(RCU_PLL2_CK)) {
        printf("RCU_PLL2_CK rcu_osci_stab_wait timeout! \r\n");
        while(1) {
        }
    }

    /* configure clock output 0 pin */
    gpio_af_set(GPIOA, GPIO_AF_0, GPIO_PIN_8);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_OSPEED_100_220MHZ, GPIO_PIN_8);

    /* configure clock output 1 pin */
    gpio_af_set(GPIOC, GPIO_AF_0, GPIO_PIN_9);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_OSPEED_100_220MHZ, GPIO_PIN_9);
}

/*!
    \brief      select type of clock output
    \param[in]  seq: sequence of clock output
    \param[out] none
    \retval     none
*/
void clock_output_select(uint8_t seq)
{
    switch(seq) {
    case CKOUT_SYSTEMCLOCK:
        rcu_ckout1_config(RCU_CKOUT1SRC_SYSTEMCLOCK, RCU_CKOUT1_DIV8);
        printf("CK_OUT1: system clock, DIV: 8 \r\n");
        break;
    case CKOUT_IRC64M:
        rcu_ckout0_config(RCU_CKOUT0SRC_IRC64MDIV,RCU_CKOUT0_DIV1);
        printf("CK_OUT0: IRC64M, DIV: 1 \r\n");
        break;
    case CKOUT_IRC48M:
        rcu_ckout0_config(RCU_CKOUT0SRC_IRC48M, RCU_CKOUT0_DIV1);
        printf("CK_OUT0: IRC48M, DIV: 1 \r\n");
        break;
    case CKOUT_IRC32K:
        rcu_ckout1_config(RCU_CKOUT1SRC_IRC32K, RCU_CKOUT1_DIV1);
        printf("CK_OUT1: IRC32K, DIV: 1 \r\n");
        break;
    case CKOUT_LXTAL:
        rcu_ckout0_config(RCU_CKOUT0SRC_LXTAL, RCU_CKOUT0_DIV1);
        printf("CK_OUT0: LXTAL, DIV: 1 \r\n");
        break;
    case CKOUT_HXTAL:
        rcu_ckout0_config(RCU_CKOUT0SRC_HXTAL, RCU_CKOUT0_DIV1);
        printf("CK_OUT0: HXTAL, DIV: 1 \r\n");
        break;
    case CKOUT_PLL1R:
        rcu_ckout1_config(RCU_CKOUT1SRC_PLL1R, RCU_CKOUT1_DIV8);
        printf("CK_OUT1: PLL1R, DIV: 8\r\n");
        break;
    case CKOUT_PLL2R:
        rcu_ckout1_config(RCU_CKOUT1SRC_PLL2R, RCU_CKOUT1_DIV8);
        printf("CK_OUT1: PLL2R, DIV: 8\r\n");
        break;

    default:
        printf("clock output select parameter error! \r\n");
        while(1) {
        }
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

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM, (uint8_t)ch);
    while(RESET == usart_flag_get(EVAL_COM, USART_FLAG_TC));

    return ch;
}
