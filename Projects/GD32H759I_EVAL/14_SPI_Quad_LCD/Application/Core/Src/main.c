/*!
    \file    main.c
    \brief   SPI QUAD LCD example

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
#include <stdio.h>
#include "GC9B71-H0189S001-320X386.h"
#include "gd32h759i_eval.h"
#include "spi_quad_lcd_driver.h"
#include "gd_logo.h"

void cache_enable(void);
void rcu_config(void);
void gpio_config(void);
void spi_config(void);

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
    /* configure systick */
    systick_config();
    /*delay 1 seconds*/
    delay_ms(1000);
    /* enable peripheral clock */
    rcu_config();
    /* configure GPIO */
    gpio_config();
    /* configure SPI */
    spi_config();
    /* initialize lcd driver GC9B71 */
    spi_quad_lcd_init();
    /* lcd display frame with cache pixel data */
    while(1)
    {
        spi_quad_lcd_picture_disp((uint8_t*)gImage_gd_logo);
        delay_ms(1000);
    }
}

/*!
    \brief      enable the CPU cache
    \param[in]  none
    \param[out] none
    \retval     none
*/
void cache_enable(void)
{
    /* enable i-cache and d-cache */
    SCB_EnableICache();
    SCB_EnableDCache();
}

/*!
    \brief      configure different peripheral clocks
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_config(void)
{
    /* enable the peripherals clock */
    rcu_periph_clock_enable(RCU_GPIOF);
    rcu_periph_clock_enable(RCU_GPIOG);
    rcu_periph_clock_enable(RCU_GPIOH);
    rcu_periph_clock_enable(RCU_SPI4);
    rcu_spi_clock_config(IDX_SPI4, RCU_SPISRC_APB2);
}

/*!
    \brief      configure the GPIO peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void gpio_config(void)
{
    /* connect port to SPI4_NSS  -> PF6
                       SPI4_SCK  -> PH6
                       SPI4_MISO -> PH7
                       SPI4_MOSI -> PF9
                       SPI4_IO2  -> PH8
                       SPI4_IO3  -> PH9 */
    gpio_af_set(GPIOF, GPIO_AF_5, GPIO_PIN_9);
    gpio_af_set(GPIOH, GPIO_AF_5, GPIO_PIN_6 | GPIO_PIN_7 |GPIO_PIN_8 | GPIO_PIN_9);

    gpio_mode_set(GPIOF, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9);
    gpio_output_options_set(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_9);

    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);

    gpio_mode_set(LCD_LEDK_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LCD_LEDK_PIN);
    gpio_output_options_set(LCD_LEDK_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, LCD_LEDK_PIN);
    gpio_bit_reset(LCD_LEDK_PORT, LCD_LEDK_PIN);

    gpio_mode_set(LCD_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LCD_CS_PIN);
    gpio_output_options_set(LCD_CS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, LCD_CS_PIN);
    gpio_bit_set(LCD_CS_PORT, LCD_CS_PIN);
}

/*!
    \brief      configure the SPI peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void spi_config(void)
{
    spi_parameter_struct spi_init_struct;
    /* deinitilize SPI and the parameters */
    spi_i2s_deinit(LCD_SPI);
    spi_struct_para_init(&spi_init_struct);

    /* SPI parameter config */
    spi_init_struct.device_mode = SPI_MASTER;
    spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.data_size = SPI_DATASIZE_8BIT;
    spi_init_struct.nss = SPI_NSS_SOFT;
    spi_init_struct.endian = SPI_ENDIAN_MSB;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
    spi_init_struct.prescale = SPI_PSC_8;
    spi_init(LCD_SPI, &spi_init_struct);

    /* enable SPI byte access */
    spi_byte_access_enable(LCD_SPI);

    /* enable SPI NSS output */
    spi_nss_output_enable(LCD_SPI);

    /* enable quad wire SPI_IO2 and SPI_IO3 pin output */
    spi_quad_io23_output_enable(LCD_SPI);

    spi_enable(LCD_SPI);
}
