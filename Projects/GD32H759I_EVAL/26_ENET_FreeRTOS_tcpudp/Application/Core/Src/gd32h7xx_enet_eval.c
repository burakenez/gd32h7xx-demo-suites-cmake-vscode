/*!
    \file    gd32h7xx_enet_eval.c
    \brief   ethernet hardware configuration

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

#include "gd32h7xx_enet.h"
#include "gd32h7xx_enet_eval.h"
#include "main.h"

static __IO uint32_t enet_init_status = 0;

static void enet_gpio_config(void);
static void enet_mac_dma_config(void);
static void nvic_configuration(void);

/*!
    \brief      setup ethernet system(GPIOs, clocks, MAC, DMA, systick)
    \param[in]  none
    \param[out] none
    \retval     none
*/
void enet_system_setup(void)
{
    nvic_configuration();

    /* configure the GPIO ports for ethernet pins */
    enet_gpio_config();

    /* configure the ethernet MAC/DMA */
    enet_mac_dma_config();

    if(0 == enet_init_status) {
        while(1) {
        }
    }


#ifdef USE_ENET0
    enet_interrupt_enable(ENET0, ENET_DMA_INT_NIE);
    enet_interrupt_enable(ENET0, ENET_DMA_INT_RIE);

#ifdef SELECT_DESCRIPTORS_ENHANCED_MODE
    enet_desc_select_enhanced_mode(ENET0);
#endif /* SELECT_DESCRIPTORS_ENHANCED_MODE */

#endif /* USE_ENET0 */
#ifdef USE_ENET1
    enet_interrupt_enable(ENET1, ENET_DMA_INT_NIE);
    enet_interrupt_enable(ENET1, ENET_DMA_INT_RIE);

#ifdef SELECT_DESCRIPTORS_ENHANCED_MODE
    enet_desc_select_enhanced_mode(ENET1);
#endif /* SELECT_DESCRIPTORS_ENHANCED_MODE */

#endif /* USE_ENET1 */
}

/*!
    \brief      configures the ethernet interface
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void enet_mac_dma_config(void)
{
    ErrStatus reval_state = ERROR;
#ifdef USE_ENET0
    /* enable ethernet clock  */
    rcu_periph_clock_enable(RCU_ENET0);
    rcu_periph_clock_enable(RCU_ENET0TX);
    rcu_periph_clock_enable(RCU_ENET0RX);

    /* reset ethernet on AHB bus */
    enet_deinit(ENET0);

    reval_state = enet_software_reset(ENET0);
    if(ERROR == reval_state) {
        while(1) {}
    }


#ifdef CHECKSUM_BY_HARDWARE
    enet_init_status = enet_init(ENET0, ENET_AUTO_NEGOTIATION, ENET_AUTOCHECKSUM_DROP_FAILFRAMES, ENET_BROADCAST_FRAMES_PASS);
#else
    enet_init_status = enet_init(ENET0, ENET_AUTO_NEGOTIATION, ENET_NO_AUTOCHECKSUM, ENET_BROADCAST_FRAMES_PASS);
#endif /* CHECKSUM_BY_HARDWARE */
#endif /* USE_ENET0 */

#ifdef USE_ENET1
    /* enable ethernet clock  */
    rcu_periph_clock_enable(RCU_ENET1);
    rcu_periph_clock_enable(RCU_ENET1TX);
    rcu_periph_clock_enable(RCU_ENET1RX);

    /* reset ethernet on AHB bus */
    enet_deinit(ENET1);

    reval_state = enet_software_reset(ENET1);
    if(ERROR == reval_state) {
        while(1) {}
    }

#ifdef CHECKSUM_BY_HARDWARE
    enet_init_status = enet_init(ENET1, ENET_AUTO_NEGOTIATION, ENET_AUTOCHECKSUM_DROP_FAILFRAMES, ENET_BROADCAST_FRAMES_PASS);
#else
    enet_init_status = enet_init(ENET1, ENET_AUTO_NEGOTIATION, ENET_NO_AUTOCHECKSUM, ENET_BROADCAST_FRAMES_PASS);
#endif /* CHECKSUM_BY_HARDWARE */
#endif /* USE_ENET1 */
}

/*!
    \brief      configures the nested vectored interrupt controller
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void nvic_configuration(void)
{
#ifdef USE_ENET0
    nvic_irq_enable(ENET0_IRQn, 2, 0);
#endif /* USE_ENET0 */
#ifdef USE_ENET1
    nvic_irq_enable(ENET1_IRQn, 2, 0);
#endif /* USE_ENET1 */
}

/*!
    \brief      configures the different GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void enet_gpio_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOE);
    rcu_periph_clock_enable(RCU_GPIOG);
    rcu_periph_clock_enable(RCU_GPIOH);

    gpio_af_set(GPIOA, GPIO_AF_0, GPIO_PIN_8);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_8);

    /* enable SYSCFG clock */
    rcu_periph_clock_enable(RCU_SYSCFG);

#ifdef MII_MODE

#ifdef PHY_CLOCK_MCO
    /* output HXTAL clock (25MHz) on CKOUT0 pin(PA8) to clock the PHY */
    rcu_ckout0_config(RCU_CKOUT0SRC_HXTAL, RCU_CKOUT0_DIV1);
#endif /* PHY_CLOCK_MCO */

#ifdef USE_ENET0
    syscfg_enet_phy_interface_config(ENET0, SYSCFG_ENET_PHY_MII);
#endif /* USE_ENET0 */
#ifdef USE_ENET1
    syscfg_enet_phy_interface_config(ENET1, SYSCFG_ENET_PHY_MII);
#endif /* USE_ENET1 */

#elif defined RMII_MODE
    /* choose DIV12 to get 50MHz from 600MHz on CKOUT0 pin (PA8) to clock the PHY */
    rcu_ckout0_config(RCU_CKOUT0SRC_PLL0P, RCU_CKOUT0_DIV12);

#ifdef USE_ENET0
    syscfg_enet_phy_interface_config(ENET0, SYSCFG_ENET_PHY_RMII);
#endif /* USE_ENET0 */
#ifdef USE_ENET1
    syscfg_enet_phy_interface_config(ENET1, SYSCFG_ENET_PHY_RMII);
#endif /* USE_ENET1 */

#endif /* MII_MODE */

#ifdef USE_ENET0
#ifdef MII_MODE

    /* PA1: ETH0_MII_RX_CLK */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_1);

    /* PA2: ETH0_MDIO */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_2);

    /* PA7: ETH0_MII_RX_DV */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_7);

    gpio_af_set(GPIOA, GPIO_AF_11, GPIO_PIN_1);
    gpio_af_set(GPIOA, GPIO_AF_11, GPIO_PIN_2);
    gpio_af_set(GPIOA, GPIO_AF_11, GPIO_PIN_7);

    /* PB8: ETH0_MII_TXD3 */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_8);

    /* PB10: ETH0_MII_RX_ER */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_10);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_10);

    gpio_af_set(GPIOB, GPIO_AF_11, GPIO_PIN_8);
    gpio_af_set(GPIOB, GPIO_AF_11, GPIO_PIN_10);

    /* PC1: ETH0_MDC */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_1);

    /* PC2: ETH0_MII_TXD2 */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_2);

    /* PC3: ETH0_MII_TX_CLK */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_3);

    /* PC4: ETH0_MII_RXD0 */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_4);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_4);

    /* PC5: ETH0_MII_RXD1 */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_5);

    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_1);
    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_2);
    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_3);
    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_4);
    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_5);

    /* PH2: ETH0_MII_CRS */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_2);

    /* PH3: ETH0_MII_COL */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_3);

    /* PH6: ETH0_MII_RXD2 */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_6);

    /* PH7: ETH0_MII_RXD3 */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_7);

    gpio_af_set(GPIOH, GPIO_AF_11, GPIO_PIN_2);
    gpio_af_set(GPIOH, GPIO_AF_11, GPIO_PIN_3);
    gpio_af_set(GPIOH, GPIO_AF_11, GPIO_PIN_6);
    gpio_af_set(GPIOH, GPIO_AF_11, GPIO_PIN_7);

    /* PG11: ETH0_MII_TX_EN */
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_11);

    /* PG13: ETH0_MII_TXD0 */
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_13);

    /* PG14: ETH0_MII_TXD1 */
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_14);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_14);

    gpio_af_set(GPIOG, GPIO_AF_11, GPIO_PIN_11);
    gpio_af_set(GPIOG, GPIO_AF_11, GPIO_PIN_13);
    gpio_af_set(GPIOG, GPIO_AF_11, GPIO_PIN_14);

    /* PD8: ETH0_INT */
    gpio_mode_set(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_8);

#elif defined RMII_MODE

    /* PA1: ETH0_RMII_REF_CLK */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_1);

    /* PA2: ETH0_MDIO */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_2);

    /* PA7: ETH0_RMII_CRS_DV */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_7);

    gpio_af_set(GPIOA, GPIO_AF_11, GPIO_PIN_1);
    gpio_af_set(GPIOA, GPIO_AF_11, GPIO_PIN_2);
    gpio_af_set(GPIOA, GPIO_AF_11, GPIO_PIN_7);

    /* PG11: ETH0_RMII_TX_EN */
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_11);

    /* PB12: ETH0_RMII_TXD0 */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_12);

    /* PG12: ETH0_RMII_TXD1 */
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_12);

    gpio_af_set(GPIOG, GPIO_AF_11, GPIO_PIN_11);
    gpio_af_set(GPIOB, GPIO_AF_11, GPIO_PIN_12);
    gpio_af_set(GPIOG, GPIO_AF_11, GPIO_PIN_12);

    /* PC1: ETH0_MDC */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_1);

    /* PC4: ETH0_RMII_RXD0 */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_4);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_4);

    /* PC5: ETH0_RMII_RXD1 */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_5);

    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_1);
    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_4);
    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_5);

#endif /* MII_MODE */
#endif /* USE_ENET0 */

#ifdef USE_ENET1
#ifdef MII_MODE

    /* PH6: ETH1_MII_RXD2 */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_6);

    /* PH7: ETH1_MII_RXD3 */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_7);

    /* PH8: ETH1_MII_RXD0 */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_8);

    /* PH9: ETH1_MII_RXD1 */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_9);

    /* PH10: ETH1_MII_RX_ER */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_10);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_10);

    /* PH11: ETH1_MII_RX_DV */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_11);

    /* PH12: ETH1_MII_RX_CLK */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_12);

    /* PH13: ETH1_MII_COL */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_13);

    /* PH14: ETH1_MDIO */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_14);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_14);

    /* PH15: ETH1_MII_CRS */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_15);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_15);

    gpio_af_set(GPIOH, GPIO_AF_6, GPIO_PIN_6);
    gpio_af_set(GPIOH, GPIO_AF_6, GPIO_PIN_7);
    gpio_af_set(GPIOH, GPIO_AF_6, GPIO_PIN_8);
    gpio_af_set(GPIOH, GPIO_AF_6, GPIO_PIN_9);
    gpio_af_set(GPIOH, GPIO_AF_6, GPIO_PIN_10);
    gpio_af_set(GPIOH, GPIO_AF_6, GPIO_PIN_11);
    gpio_af_set(GPIOH, GPIO_AF_6, GPIO_PIN_12);
    gpio_af_set(GPIOH, GPIO_AF_6, GPIO_PIN_13);
    gpio_af_set(GPIOH, GPIO_AF_6, GPIO_PIN_14);
    gpio_af_set(GPIOH, GPIO_AF_6, GPIO_PIN_15);

    /* PG6: ETH1_MDC */
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_6);

    /* PG9: ETH1_MII_TX_CLK */
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_9);

    /* PG11: ETH1_MII_TX_EN */
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_11);

    /* PG12: ETH1_MII_TXD2 */
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_12);

    /* PG13: ETH1_MII_TXD0 */
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_13);

    /* PG14: ETH1_MII_TXD1 */
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_14);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_14);

    /* PG15: ETH1_MII_TXD3 */
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_15);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_15);

    gpio_af_set(GPIOG, GPIO_AF_6, GPIO_PIN_6);
    gpio_af_set(GPIOG, GPIO_AF_6, GPIO_PIN_9);
    gpio_af_set(GPIOG, GPIO_AF_6, GPIO_PIN_11);
    gpio_af_set(GPIOG, GPIO_AF_6, GPIO_PIN_12);
    gpio_af_set(GPIOG, GPIO_AF_6, GPIO_PIN_13);
    gpio_af_set(GPIOG, GPIO_AF_6, GPIO_PIN_14);
    gpio_af_set(GPIOG, GPIO_AF_6, GPIO_PIN_15);

    /* PE1: ETH1_INT */
    gpio_mode_set(GPIOE, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_1);

#elif defined RMII_MODE

    /* PH8: ETH1_RMII_RXD0 */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_8);

    /* PH9: ETH1_RMII_RXD1 */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_9);

    /* PH11: ETH1_RMII_CRS_DV */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_11);

    /* PH12: ETH1_RMII_REF_CLK */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_12);

    /* PH14: ETH1_MDIO */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_14);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_14);


    gpio_af_set(GPIOH, GPIO_AF_6, GPIO_PIN_8);
    gpio_af_set(GPIOH, GPIO_AF_6, GPIO_PIN_9);
    gpio_af_set(GPIOH, GPIO_AF_6, GPIO_PIN_11);
    gpio_af_set(GPIOH, GPIO_AF_6, GPIO_PIN_12);
    gpio_af_set(GPIOH, GPIO_AF_6, GPIO_PIN_14);

    /* PG6: ETH1_MDC */
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_6);

    /* PG11: ETH1_RMII_TX_EN */
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_11);

    /* PG13: ETH1_RMII_TXD0 */
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_13);

    /* PG14: ETH1_RMII_TXD1 */
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_14);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_14);

    gpio_af_set(GPIOG, GPIO_AF_6, GPIO_PIN_6);
    gpio_af_set(GPIOG, GPIO_AF_6, GPIO_PIN_11);
    gpio_af_set(GPIOG, GPIO_AF_6, GPIO_PIN_13);
    gpio_af_set(GPIOG, GPIO_AF_6, GPIO_PIN_14);

#endif /* MII_MODE */
#endif /* USE_ENET1 */
}
