/*!
    \file    main.c
    \brief   HPDF SAI audio demo

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
#include "systick.h"
#include <stdio.h>
#include "gd32h759i_eval.h"

#define BUFFER_SIZE           3072
#define PLAY_BUFFER_SIZE      6144
#define CKOUTDIV_25           25
#define FLT_OVER_SAMPLE_64    64

/* limit the range of data */
#define DATA_LIMIT(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))

/* PDM left channel data */
__attribute__((aligned(32))) int32_t pcm_left_data[BUFFER_SIZE];
/* PDM right channel data */
__attribute__((aligned(32))) int32_t pcm_right_data[BUFFER_SIZE];
/* audio data  */
__attribute__((aligned(32))) int16_t pcm_play_data[PLAY_BUFFER_SIZE] = {0};

void rcu_config(void);
void gpio_config(void);
void dma_config(void);
void spi_config(void);
void hpdf_config(void);
void sai_config(void);
void cache_enable(void);
uint32_t temp = 0;

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    uint32_t i;

    /* enable the cache */
    cache_enable();
    /* enable interrupt */
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    /* NVIC configuration */
    nvic_irq_enable(SAI1_IRQn, 1, 1);

    /* configure RCU */
    rcu_config();
    /* configure GPIO */
    gpio_config();
    /* configure DMA */
    dma_config();
    /* configure SAI */
    sai_config();
    /* configure HPDF */
    hpdf_config();

    /* enable the SAI interrupt */
    sai_interrupt_enable(SAI1, SAI_BLOCK1, SAI_INT_FFREQ);

    while(1) {
        SCB_CleanInvalidateDCache();
        /* wait for DMA half-full transmit complete */
        while(RESET == dma_flag_get(DMA1, DMA_CH1, DMA_FLAG_HTF));
        while(RESET == dma_flag_get(DMA1, DMA_CH2, DMA_FLAG_HTF));

        /* get the PCM stereo data */
        for(i = 0; i < BUFFER_SIZE / 2; i++) {
            pcm_play_data[2 * i] = DATA_LIMIT((pcm_left_data[i] >> 9), -32768, 32767);
            pcm_play_data[(2 * i) + 1] = DATA_LIMIT((pcm_right_data[i] >> 9), -32768, 32767);
        }
        /* clear the half transfer finish flag */
        dma_flag_clear(DMA1, DMA_CH1, DMA_FLAG_HTF);
        dma_flag_clear(DMA1, DMA_CH2, DMA_FLAG_HTF);

        /* wait for DMA full transmit complete */
        while(RESET == dma_flag_get(DMA1, DMA_CH1, DMA_FLAG_FTF));
        while(RESET == dma_flag_get(DMA1, DMA_CH2, DMA_FLAG_FTF));

        /* get the PCM stereo data */
        for(i = BUFFER_SIZE / 2; i < BUFFER_SIZE; i++) {
            pcm_play_data[2 * i] = DATA_LIMIT((pcm_left_data[i] >> 9), -32768, 32767);
            pcm_play_data[(2 * i) + 1] = DATA_LIMIT((pcm_right_data[i] >> 9), -32768, 32767);
        }
        /* clear the full transfer finish flag */
        dma_flag_clear(DMA1, DMA_CH1, DMA_FLAG_FTF);
        dma_flag_clear(DMA1, DMA_CH2, DMA_FLAG_FTF);
    }
}

/*!
    \brief      configure different peripheral clocks
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_config(void)
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOH);
    rcu_periph_clock_enable(RCU_GPIOG);
    /* SAI_CK = PLL1_VCO/PLL1P = 200/25 = 8 Mhz */
    rcu_pll_input_output_clock_range_config(IDX_PLL1, RCU_PLL1RNG_1M_2M, RCU_PLL1VCO_192M_836M);
    rcu_pll1_config(25, 200, 25, 25, 25);
    rcu_sai_clock_config(IDX_SAI1, RCU_SAISRC_PLL1P);
    rcu_pll_clock_output_enable(RCU_PLL1P);
    rcu_osci_on(RCU_PLL1_CK);
    if(ERROR == rcu_osci_stab_wait(RCU_PLL1_CK)) {
        while(1) {
        }
    }

    /* enable SAI clock */
    rcu_periph_clock_enable(RCU_SAI1);

    /* enable DMA0 clock */
    rcu_periph_clock_enable(RCU_DMA0);
    /* enable DMA1 clock */
    rcu_periph_clock_enable(RCU_DMA1);
    rcu_periph_clock_enable(RCU_DMAMUX);

    /* select HPDF clock source */
    rcu_hpdf_clock_config(RCU_HPDFSRC_APB2);
    /* enable HPDF clock */
    rcu_periph_clock_enable(RCU_HPDF);
}

/*!
    \brief      configure the GPIO peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void gpio_config(void)
{
    /* connect PB0 to HPDF_CKOUT */
    gpio_af_set(GPIOB, GPIO_AF_6, GPIO_PIN_0);
    /* connect PB1 to HPDF_DATA1 */
    gpio_af_set(GPIOB, GPIO_AF_6, GPIO_PIN_1);
    /* configure GPIO pins of HPDF */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_0);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_1);

    /* configure GPIO pins of SAI1: SAI1_MCLK1(PH3) SAI1_FS1(PG9) SAI1_SCK1(PH2) SAI1_SD1(PG10) */
    gpio_af_set(GPIOG, GPIO_AF_10, GPIO_PIN_9 | GPIO_PIN_10);
    gpio_af_set(GPIOH, GPIO_AF_10, GPIO_PIN_2 | GPIO_PIN_3);
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE,
                  GPIO_PIN_9 | GPIO_PIN_10);
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE,
                  GPIO_PIN_2 | GPIO_PIN_3);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ,
                            GPIO_PIN_9 | GPIO_PIN_10);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ,
                            GPIO_PIN_2 | GPIO_PIN_3);
}

/*!
    \brief      configure the HPDF
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hpdf_config(void)
{
    hpdf_channel_parameter_struct hpdf_channel_init_struct;
    hpdf_filter_parameter_struct hpdf_filter_init_struct;
    hpdf_rc_parameter_struct hpdf_rc_init_struct;

    /* reset HPDF */
    hpdf_deinit();

    /*  initialize the parameters */
    hpdf_channel_struct_para_init(&hpdf_channel_init_struct);
    hpdf_filter_struct_para_init(&hpdf_filter_init_struct);
    hpdf_rc_struct_para_init(&hpdf_rc_init_struct);

    /* configure serial clock output */
    rcu_sai_clock_config(IDX_SAI0, RCU_SAISRC_PER);
    rcu_per_clock_config(RCU_PERSRC_HXTAL);
    hpdf_clock_output_config(SERIAL_AUDIO_CLK, CKOUTDIV_25, CKOUTDM_ENABLE);

    /* initialize HPDF channel0 */
    hpdf_channel_init_struct.data_packing_mode      = DPM_STANDARD_MODE;
    hpdf_channel_init_struct.channel_pin_select     = CHPINSEL_NEXT;
    hpdf_channel_init_struct.ck_loss_detector       = CLK_LOSS_DISABLE;
    hpdf_channel_init_struct.malfunction_monitor    = MM_ENABLE;
    hpdf_channel_init_struct.spi_ck_source          = INTERNAL_CKOUT;
    hpdf_channel_init_struct.channel_multiplexer    = SERIAL_INPUT;
    hpdf_channel_init_struct.serial_interface       = SPI_FALLING_EDGE;
    hpdf_channel_init_struct.calibration_offset     = 0;
    hpdf_channel_init_struct.right_bit_shift        = 0x06;
    hpdf_channel_init_struct.tm_filter              = TM_FASTSINC;
    hpdf_channel_init_struct.tm_filter_oversample   = TM_FLT_BYPASS;
    hpdf_channel_init_struct.mm_break_signal        = DISABLE;
    hpdf_channel_init_struct.mm_counter_threshold   = 255;
    hpdf_channel_init_struct.plsk_value             = 0;
    hpdf_channel_init(CHANNEL0, &hpdf_channel_init_struct);

    /* initialize HPDF channel1 */
    hpdf_channel_init_struct.channel_pin_select     = CHPINSEL_CURRENT;
    hpdf_channel_init_struct.spi_ck_source          = INTERNAL_CKOUT;
    hpdf_channel_init_struct.serial_interface       = SPI_RISING_EDGE;
    hpdf_channel_init_struct.tm_filter              = TM_FASTSINC;
    hpdf_channel_init_struct.tm_filter_oversample   = TM_FLT_BYPASS;
    hpdf_channel_init_struct.mm_counter_threshold   = 255;
    hpdf_channel_init_struct.plsk_value             = 0;
    hpdf_channel_init_struct.right_bit_shift        = 0x06;
    hpdf_channel_init(CHANNEL1, &hpdf_channel_init_struct);

    /* initialize HPDF filter0 and filter1 */
    hpdf_filter_init_struct.sinc_filter             = FLT_SINC4;
    hpdf_filter_init_struct.sinc_oversample         = FLT_OVER_SAMPLE_64;
    hpdf_filter_init_struct.integrator_oversample   = INTEGRATOR_BYPASS;
    hpdf_filter_init(FLT0, &hpdf_filter_init_struct);
    hpdf_filter_init(FLT1, &hpdf_filter_init_struct);

    /* initialize HPDF filter0 regular conversions */
    hpdf_rc_init_struct.fast_mode       = FAST_ENABLE;
    hpdf_rc_init_struct.rcs_channel     = RCS_CHANNEL0;
    hpdf_rc_init_struct.rcdmaen         = RCDMAEN_ENABLE;
    hpdf_rc_init_struct.continuous_mode = RCCM_ENABLE;
    hpdf_rc_init(FLT0, &hpdf_rc_init_struct);

    /* initialize HPDF filter1 regular conversions */
    hpdf_rc_init_struct.fast_mode       = FAST_ENABLE;
    hpdf_rc_init_struct.rcs_channel     = RCS_CHANNEL1;
    hpdf_rc_init_struct.rcdmaen         = RCDMAEN_ENABLE;
    hpdf_rc_init_struct.continuous_mode = RCCM_ENABLE;
    hpdf_rc_init(FLT1, &hpdf_rc_init_struct);

    /* enable channel */
    hpdf_channel_enable(CHANNEL0);
    hpdf_channel_enable(CHANNEL1);
    /* enable filter */
    hpdf_filter_enable(FLT0);
    hpdf_filter_enable(FLT1);
    /* enable the HPDF module globally */
    hpdf_enable();
    /* enable regular channel conversion by software */
    hpdf_rc_start_by_software(FLT0);
    hpdf_rc_start_by_software(FLT1);
}

/*!
    \brief      configure DMA
    \param[in]  none
    \param[out] none
    \retval     none
*/
void dma_config(void)
{
    dma_single_data_parameter_struct dma_init_parameter;
    dma_single_data_para_struct_init(&dma_init_parameter);

    /* deinitialize DMA1_CH1 */
    dma_deinit(DMA1, DMA_CH1);
    /* configure DMA1_CH1 */
    dma_init_parameter.request = DMA_REQUEST_HPDF_FLT0;
    dma_init_parameter.periph_addr         = (int32_t)&HPDF_FLTYRDATA(FLT0);
    dma_init_parameter.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_parameter.memory0_addr        = (uint32_t)pcm_right_data;
    dma_init_parameter.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_parameter.periph_memory_width = DMA_PERIPH_WIDTH_32BIT;
    dma_init_parameter.circular_mode       = DMA_CIRCULAR_MODE_ENABLE;
    dma_init_parameter.direction           = DMA_PERIPH_TO_MEMORY;
    dma_init_parameter.number              = BUFFER_SIZE;
    dma_init_parameter.priority            = DMA_PRIORITY_ULTRA_HIGH;
    dma_single_data_mode_init(DMA1, DMA_CH1, &dma_init_parameter);
    /* enable DMA channel */
    dma_channel_enable(DMA1, DMA_CH1);

    /* deinitialize DMA1_CH2 */
    dma_deinit(DMA1, DMA_CH2);
    /* configure DMA1_CH2 */
    dma_init_parameter.request = DMA_REQUEST_HPDF_FLT1;
    dma_init_parameter.periph_addr  = (int32_t)&HPDF_FLTYRDATA(FLT1);
    dma_init_parameter.memory0_addr = (uint32_t)pcm_left_data;
    dma_init_parameter.priority     = DMA_PRIORITY_HIGH;
    dma_single_data_mode_init(DMA1, DMA_CH2, &dma_init_parameter);
    /* enable DMA channel */
    dma_channel_enable(DMA1, DMA_CH2);
}

/*!
    \brief      configure the SAI peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void sai_config(void)
{
    sai_parameter_struct sai_structure;
    sai_frame_parameter_struct sai_frame_structure;
    sai_slot_parameter_struct sai_slot_structure;

    sai_struct_para_init(&sai_structure);
    sai_frame_struct_para_init(&sai_frame_structure);
    sai_slot_struct_para_init(&sai_slot_structure);

    /* initialize SAI1_B1 frame */
    sai_frame_structure.frame_width            = 32;
    sai_frame_structure.frame_sync_width       = 16;
    sai_frame_structure.frame_sync_function    = SAI_FS_FUNC_START_CHANNEL;
    sai_frame_structure.frame_sync_polarity    = SAI_FS_POLARITY_LOW;
    sai_frame_structure.frame_sync_offset      = SAI_FS_OFFSET_BEGINNING;
    sai_frame_init(SAI1, SAI_BLOCK1, &sai_frame_structure);

    /* initialize SAI1_B1 slot */
    sai_slot_structure.slot_number             = 2;
    sai_slot_structure.slot_width              = SAI_SLOT_WIDTH_DATA;
    sai_slot_structure.data_offset             = 0;
    sai_slot_structure.slot_active             = SAI_SLOT_ACTIVE_ALL;
    sai_slot_init(SAI1, SAI_BLOCK1, &sai_slot_structure);

    /* initialize SAI1_B1  */
    sai_structure.operating_mode               = SAI_MASTER_TRANSMITTER;
    sai_structure.protocol                     = SAI_PROTOCOL_POLYMORPHIC;
    sai_structure.data_width                   = SAI_DATAWIDTH_16BIT;
    sai_structure.shift_dir                    = SAI_SHIFT_MSB;
    sai_structure.sample_edge                  = SAI_SAMPEDGE_RISING;
    sai_structure.sync_mode                    = SAI_SYNCMODE_ASYNC;
    sai_structure.output_drive                 = SAI_OUTPUT_WITH_SAIEN;
    sai_structure.clk_div_bypass               = SAI_CLKDIV_BYPASS_OFF;
    sai_structure.mclk_div                     = SAI_MCLKDIV_2;
    sai_structure.mclk_oversampling            = SAI_MCLK_OVERSAMP_256;
    sai_structure.mclk_enable                  = SAI_MCLK_ENABLE;
    sai_structure.fifo_threshold               = SAI_FIFOTH_HALF;
    sai_init(SAI1, SAI_BLOCK1, &sai_structure);
    /* sai enable*/
    sai_enable(SAI1, SAI_BLOCK1);
}

/*!
    \brief      enable the CPU cache
    \param[in]  none
    \param[out] none
    \retval     none
*/
void cache_enable(void)
{
    /* enable i-cache d-cache */
    SCB_EnableICache();
    SCB_EnableDCache();
}
