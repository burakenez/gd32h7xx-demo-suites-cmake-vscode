/*!
    \file    main.c
    \brief   touch the screen to draw dots

    \version 2024-09-25, V1.0.0, demo for GD32H7xx
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
#include "gd32h759i_eval.h"
#include "bsp_ts_gt911.h"
#include "stdlib.h"

#define HORIZONTAL_SYNCHRONOUS_PULSE  41
#define HORIZONTAL_BACK_PORCH         2
#define ACTIVE_WIDTH                  480
#define HORIZONTAL_FRONT_PORCH        2

#define VERTICAL_SYNCHRONOUS_PULSE    10
#define VERTICAL_BACK_PORCH           2
#define ACTIVE_HEIGHT                 272
#define VERTICAL_FRONT_PORCH          2
#define TOUCH_POINT_SIZE              1
#define FRAME_BUFFER_START_ADDR       0x24005000

/* pointer to the starting address of the framebuffer in memory */
uint16_t  *framebuffer = (uint16_t *)FRAME_BUFFER_START_ADDR;

/* function prototypes */
static void tli_gpio_config(void);
static void tli_config(void);
static void cache_enable();
void draw_point(uint16_t x, uint16_t y, uint16_t color);
void framebuffer_init(void);
void handle_touch(void);

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
    /* configure TLI */
    tli_config();
    framebuffer_init();

    tli_layer_enable(LAYER0);
    /* reload layer0 and layer1 configuration */
    tli_reload_config(TLI_REQUEST_RELOAD_EN);
    tli_enable();

    gt911_init();
    gt911_read_id();
    while(1) {
        SCB_CleanInvalidateDCache();
        handle_touch();
    }
}

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
    \brief      draw a point on the screen
    \param[in]  x: x-coordinate of the point
    \param[in]  y: y-coordinate of the point
    \param[in]  color: color of the point in RGB565 format
    \param[out] none
    \retval     none
*/
void draw_point(uint16_t x, uint16_t y, uint16_t color)
{
    if(x >= ACTIVE_WIDTH || y >= ACTIVE_HEIGHT) {
        return;
    }

    /* draw a simple square dot */
    for(int i = -TOUCH_POINT_SIZE / 2; i <= TOUCH_POINT_SIZE / 2; i++) {
        for(int j = -TOUCH_POINT_SIZE / 2; j <= TOUCH_POINT_SIZE / 2; j++) {
            if(x + i < ACTIVE_WIDTH && y + j < ACTIVE_HEIGHT) {
                framebuffer[(y + j) * ACTIVE_WIDTH + (x + i)] = color;
            }
        }
    }
}

/*!
    \brief      initialize the framebuffer with a default color
    \param[in]  none
    \param[out] none
    \retval     none
*/
void framebuffer_init(void)
{
    uint32_t volatile x, y;
    for(uint32_t  y = 0; y < ACTIVE_HEIGHT; ++y) {
        for(uint32_t  x = 0; x < ACTIVE_WIDTH; ++x) {
            framebuffer[y * ACTIVE_WIDTH + x] = 0xFFFF;
        }
    }
    SCB_CleanInvalidateDCache();
}

/*!
    \brief      handle touch input and draw points on the screen
    \param[in]  none
    \param[out] none
    \retval     none
*/
void handle_touch(void)
{
    uint16_t x, y;
    if(gt911_handler() == SUCCESS && gt911.pressed_info & 0x0F) {
        for(uint8_t i = 0; i < (gt911.pressed_info & 0x0F); i++) {
            x = gt911.x[i];
            y = gt911.y[i];
            /* red color in RGB565 */
            draw_point(x, y, 0xF800U);
        }
    }
}

/*!
    \brief      LCD configuration
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void tli_config(void)
{
    tli_parameter_struct               tli_init_struct;
    tli_layer_parameter_struct         tli_layer_init_struct;

    rcu_periph_clock_enable(RCU_TLI);
    tli_gpio_config();

    /* configure PLL2 to generate TLI clock 25MHz/25*192/3 = 64MHz*/
    rcu_pll_input_output_clock_range_config(IDX_PLL2, RCU_PLL2RNG_1M_2M, RCU_PLL2VCO_192M_836M);
    if(ERROR == rcu_pll2_config(25, 192, 3, 3, 3)) {
        while(1) {
        }
    }
    rcu_pll_clock_output_enable(RCU_PLL2R);
    rcu_tli_clock_div_config(RCU_PLL2R_DIV8);

    rcu_osci_on(RCU_PLL2_CK);

    if(ERROR == rcu_osci_stab_wait(RCU_PLL2_CK)) {
        while(1) {
        }
    }

    /* configure TLI parameter struct */
    tli_init_struct.signalpolarity_hs = TLI_HSYN_ACTLIVE_LOW;
    tli_init_struct.signalpolarity_vs = TLI_VSYN_ACTLIVE_LOW;
    tli_init_struct.signalpolarity_de = TLI_DE_ACTLIVE_LOW;
    tli_init_struct.signalpolarity_pixelck = TLI_PIXEL_CLOCK_TLI;
    /* LCD display timing configuration */
    tli_init_struct.synpsz_hpsz = HORIZONTAL_SYNCHRONOUS_PULSE - 1;
    tli_init_struct.synpsz_vpsz = VERTICAL_SYNCHRONOUS_PULSE - 1;
    tli_init_struct.backpsz_hbpsz = HORIZONTAL_SYNCHRONOUS_PULSE + HORIZONTAL_BACK_PORCH - 1;
    tli_init_struct.backpsz_vbpsz = VERTICAL_SYNCHRONOUS_PULSE + VERTICAL_BACK_PORCH - 1;
    tli_init_struct.activesz_hasz = HORIZONTAL_SYNCHRONOUS_PULSE + HORIZONTAL_BACK_PORCH + ACTIVE_WIDTH
                                    - 1;
    tli_init_struct.activesz_vasz = VERTICAL_SYNCHRONOUS_PULSE + VERTICAL_BACK_PORCH + ACTIVE_HEIGHT -
                                    1;
    tli_init_struct.totalsz_htsz = HORIZONTAL_SYNCHRONOUS_PULSE + HORIZONTAL_BACK_PORCH + ACTIVE_WIDTH +
                                   HORIZONTAL_FRONT_PORCH - 1;
    tli_init_struct.totalsz_vtsz = VERTICAL_SYNCHRONOUS_PULSE + VERTICAL_BACK_PORCH + ACTIVE_HEIGHT +
                                   VERTICAL_FRONT_PORCH - 1;
    tli_init_struct.backcolor_blue = 0;
    tli_init_struct.backcolor_green = 0;
    tli_init_struct.backcolor_red = 0;
    tli_init(&tli_init_struct);

    /* TLI layer0 configuration */
    /* TLI window size configuration */
    tli_layer_init_struct.layer_window_leftpos = HORIZONTAL_SYNCHRONOUS_PULSE + HORIZONTAL_BACK_PORCH;
    tli_layer_init_struct.layer_window_rightpos = (480 + HORIZONTAL_SYNCHRONOUS_PULSE +
                                                   HORIZONTAL_BACK_PORCH - 1);
    tli_layer_init_struct.layer_window_toppos = VERTICAL_SYNCHRONOUS_PULSE + VERTICAL_BACK_PORCH;
    tli_layer_init_struct.layer_window_bottompos = (272 + VERTICAL_SYNCHRONOUS_PULSE +
                                                    VERTICAL_BACK_PORCH - 1);
    /* TLI window pixel format configuration */
    tli_layer_init_struct.layer_ppf = LAYER_PPF_RGB565;
    /* TLI window specified alpha configuration */
    tli_layer_init_struct.layer_sa = 255;
    /* TLI layer default alpha R,G,B value configuration */
    tli_layer_init_struct.layer_default_blue = 0;
    tli_layer_init_struct.layer_default_green = 0;
    tli_layer_init_struct.layer_default_red = 0;
    tli_layer_init_struct.layer_default_alpha = 0;
    /* TLI window blend configuration */
    tli_layer_init_struct.layer_acf1 = LAYER_ACF1_SA;
    tli_layer_init_struct.layer_acf2 = LAYER_ACF2_SA;
    /* TLI layer frame buffer base address configuration */
    tli_layer_init_struct.layer_frame_bufaddr = FRAME_BUFFER_START_ADDR;
    tli_layer_init_struct.layer_frame_line_length = ((480 * 2) + 7);
    tli_layer_init_struct.layer_frame_buf_stride_offset = (480 * 2);
    tli_layer_init_struct.layer_frame_total_line_number = 272;
    tli_layer_init(LAYER0, &tli_layer_init_struct);
    tli_dither_config(TLI_DITHER_ENABLE);
}

/*!
    \brief      configure TLI GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void tli_gpio_config(void)
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOE);
    rcu_periph_clock_enable(RCU_GPIOF);
    rcu_periph_clock_enable(RCU_GPIOH);
    rcu_periph_clock_enable(RCU_GPIOG);

    /* configure HSYNC(PE15), VSYNC(PA7), PCLK(PG7) */
    gpio_af_set(GPIOE, GPIO_AF_10, GPIO_PIN_15);
    gpio_af_set(GPIOA, GPIO_AF_14, GPIO_PIN_7);
    gpio_af_set(GPIOG, GPIO_AF_14, GPIO_PIN_7);
    gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_15);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_15);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_7);
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_7);

    /* configure LCD_R7(PG6), LCD_R6(PH12), LCD_R5(PH11), LCD_R4(PA5), LCD_R3(PH9),LCD_R2(PH8),
                 LCD_R1(PH3), LCD_R0(PH2) */
    gpio_af_set(GPIOG, GPIO_AF_14, GPIO_PIN_6);
    gpio_af_set(GPIOH, GPIO_AF_14, GPIO_PIN_12);
    gpio_af_set(GPIOH, GPIO_AF_14, GPIO_PIN_11);
    gpio_af_set(GPIOA, GPIO_AF_14, GPIO_PIN_5);
    gpio_af_set(GPIOH, GPIO_AF_14, GPIO_PIN_9);
    gpio_af_set(GPIOH, GPIO_AF_14, GPIO_PIN_8);
    gpio_af_set(GPIOH, GPIO_AF_14, GPIO_PIN_3);
    gpio_af_set(GPIOH, GPIO_AF_14, GPIO_PIN_2);

    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_6);
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_12);
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_11);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_5);
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_9);
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_8);
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_3);
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_2);

    /* configure  LCD_G7(PD3), LCD_G6(PC7), LCD_G5(PC1), LCD_G4(PH15), LCD_G3(PH14), LCD_G2(PH13),LCD_G1(PB0), LCD_G0(PB1) */
    gpio_af_set(GPIOD, GPIO_AF_14, GPIO_PIN_3);
    gpio_af_set(GPIOC, GPIO_AF_14, GPIO_PIN_7);
    gpio_af_set(GPIOC, GPIO_AF_14, GPIO_PIN_1);
    gpio_af_set(GPIOH, GPIO_AF_14, GPIO_PIN_15);
    gpio_af_set(GPIOH, GPIO_AF_14, GPIO_PIN_14);
    gpio_af_set(GPIOH, GPIO_AF_14, GPIO_PIN_13);
    gpio_af_set(GPIOB, GPIO_AF_14, GPIO_PIN_0);
    gpio_af_set(GPIOB, GPIO_AF_14, GPIO_PIN_1);

    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_3);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_7);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_1);
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_15);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_15);
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_14);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_14);
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_13);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_0);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_1);

    /* configure LCD_B7(PB9), LCD_B6(PB8), LCD_B5(PB5), LCD_B4(PC11), LCD_B3(PG11),LCD_B2(PG10), LCD_B1(PG12), LCD_B0(PG14) */
    gpio_af_set(GPIOB, GPIO_AF_14, GPIO_PIN_9);
    gpio_af_set(GPIOB, GPIO_AF_14, GPIO_PIN_8);
    gpio_af_set(GPIOB, GPIO_AF_3, GPIO_PIN_5);
    gpio_af_set(GPIOC, GPIO_AF_14, GPIO_PIN_11);
    gpio_af_set(GPIOG, GPIO_AF_14, GPIO_PIN_11);
    gpio_af_set(GPIOG, GPIO_AF_14, GPIO_PIN_10);
    gpio_af_set(GPIOG, GPIO_AF_14, GPIO_PIN_12);
    gpio_af_set(GPIOG, GPIO_AF_14, GPIO_PIN_14);

    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_9);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_8);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_5);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_11);
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_11);
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_10);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_10);
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_12);
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_14);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_14);

    /* configure LCD_DE(PF10) */
    gpio_af_set(GPIOF, GPIO_AF_14, GPIO_PIN_10);
    gpio_mode_set(GPIOF, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_10);
    gpio_output_options_set(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_10);
    /* LCD PWM BackLight(PG13) */
    gpio_mode_set(GPIOG, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_13);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_13);
    gpio_bit_set(GPIOG, GPIO_PIN_13);
}
