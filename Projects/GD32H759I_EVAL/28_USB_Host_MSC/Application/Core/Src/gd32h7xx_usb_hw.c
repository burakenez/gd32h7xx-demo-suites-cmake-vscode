/*!
    \file    gd32h7xx_usb_hw.c
    \brief   USB hardware configuration for GD32H7xx

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

#include "drv_usb_regs.h"
#include "drv_usb_hw.h"

#define TIM_MSEC_DELAY                          0x01U
#define TIM_USEC_DELAY                          0x02U

#ifndef USE_ULPI_PHY

#ifdef USE_USBHS0
#define HOST_POWERSW_PORT_RCC                   RCU_GPIOA
#define HOST_POWERSW_PORT                       GPIOA
#define HOST_POWERSW_VBUS                       GPIO_PIN_6
#endif

#ifdef USE_USBHS1
#define HOST_POWERSW_PORT_RCC                   RCU_GPIOB
#define HOST_POWERSW_PORT                       GPIOB
#define HOST_POWERSW_VBUS                       GPIO_PIN_10
#endif

#else

#ifdef USE_USBHS0
#define HOST_POWERSW_PORT_RCC                   RCU_GPIOC
#define HOST_POWERSW_PORT                       GPIOC
#define HOST_POWERSW_VBUS                       GPIO_PIN_7
#endif

#ifdef USE_USBHS1
#define HOST_POWERSW_PORT_RCC                   RCU_GPIOC
#define HOST_POWERSW_PORT                       GPIOC
#define HOST_POWERSW_VBUS                       GPIO_PIN_9
#endif 

#endif

#define HOST_SOF_OUTPUT_RCC                     RCC_APB2PERIPH_GPIOA
#define HOST_SOF_PORT                           GPIOA
#define HOST_SOF_SIGNAL                         GPIO_PIN_8

__IO uint32_t delay_time = 0U;
__IO uint16_t timer_prescaler = 29U;

/* local function prototypes ('static') */
static void hw_time_set(uint8_t unit);
static void hw_delay(uint32_t ntime, uint8_t unit);

/*!
    \brief      configure USB GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_gpio_config(void)
{
#ifdef USE_USBHS0
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOF);

    /* ULPI_STP(PC0) GPIO pin configuration */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_0);

    /* ULPI_CK(PA5) GPIO pin configuration */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_5);

    /* ULPI_NXT(PC3) GPIO pin configuration */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_3);

    /* ULPI_DIR(PI11) GPIO pin configuration */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_2);

    /* ULPI_D1(PB0), ULPI_D2(PB1), ULPI_D3(PB10), ULPI_D4(PB11) \
       ULPI_D5(PB12), ULPI_D6(PB13) and ULPI_D7(PB5) GPIO pin configuration */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, \
                  GPIO_PIN_5 | GPIO_PIN_10 | GPIO_PIN_1 | GPIO_PIN_0);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, \
                            GPIO_PIN_5 | GPIO_PIN_10 | GPIO_PIN_1 | GPIO_PIN_0);

    gpio_mode_set(GPIOF, GPIO_MODE_AF, GPIO_PUPD_NONE, \
                    GPIO_PIN_2 |\
                    GPIO_PIN_1 | GPIO_PIN_0);
    gpio_output_options_set(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, \
                    GPIO_PIN_2   |\
                    GPIO_PIN_1 | GPIO_PIN_0);

    /* ULPI_D0(PA3) GPIO pin configuration */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_3);
    

    gpio_af_set(GPIOF, GPIO_AF_5, GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_1);
    gpio_af_set(GPIOC, GPIO_AF_10, GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3);
    gpio_af_set(GPIOA, GPIO_AF_10, GPIO_PIN_5 | GPIO_PIN_3);
    gpio_af_set(GPIOB, GPIO_AF_10, GPIO_PIN_5  |\
                                    GPIO_PIN_10 | GPIO_PIN_1 | GPIO_PIN_0);
#endif

#ifdef USE_USBHS1 
    rcu_periph_clock_enable(RCU_GPIOH);
    rcu_periph_clock_enable(RCU_GPIOG);

    /* ULPI_STP(PH2) GPIO pin configuration */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_2);

    /* ULPI_CK(PH5) GPIO pin configuration */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_5);

    /* ULPI_NXT(PH4) GPIO pin configuration */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_4);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_4);

    /* ULPI_DIR(PH3) GPIO pin configuration */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_3);

    /* ULPI_D0(PH6) GPIO pin configuration */
    /* ULPI_D1(PH7), ULPI_D2(PH8), ULPI_D3(PH9), ULPI_D4(PH10) \
       ULPI_D5(PH11), ULPI_D6(PH12) and ULPI_D7(PG5) GPIO pin configuration */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, \
                    GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 |\
                    GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, \
                    GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 |\
                    GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);

    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_5);

    gpio_af_set(GPIOG, GPIO_AF_8, GPIO_PIN_5);
    gpio_af_set(GPIOH, GPIO_AF_8, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |\
                                   GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8| GPIO_PIN_9| GPIO_PIN_10| GPIO_PIN_11| GPIO_PIN_12);
#endif
}

/*!
    \brief      configure USB clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_rcu_config(void)
{
    pmu_usb_regulator_enable();
    pmu_usb_voltage_detector_enable();
    while(SET != pmu_flag_get(PMU_FLAG_USB33RF)) {
    }

#ifdef USE_USB_FS

#ifndef USE_IRC48M
    /* configure the pll1 input and output clock range */
    rcu_pll_input_output_clock_range_config(IDX_PLL1,RCU_PLL1RNG_4M_8M,RCU_PLL1VCO_192M_836M);

    rcu_pll1_config(5, 96, 2, 10, 2);

    /* enable PLL1Q clock output */
    rcu_pll_clock_output_enable(RCU_PLL1Q);
    rcu_osci_on(RCU_PLL1_CK);

#ifdef USE_USBHS0
    rcu_usbhs_pll1qpsc_config(IDX_USBHS0, RCU_USBHSPSC_DIV1);

    rcu_usb48m_clock_config(IDX_USBHS0, RCU_USB48MSRC_PLL1Q);
#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
    rcu_usbhs_pll1qpsc_config(IDX_USBHS1, RCU_USBHSPSC_DIV1);

    rcu_usb48m_clock_config(IDX_USBHS1, RCU_USB48MSRC_PLL1Q);
#endif /* USE_USBHS1 */

#else
    /* enable IRC48M clock */
    rcu_osci_on(RCU_IRC48M);

    /* wait till IRC48M is ready */
    while(SUCCESS != rcu_osci_stab_wait(RCU_IRC48M)) {
    }

    rcu_ck48m_clock_config(RCU_CK48MSRC_IRC48M);

#ifdef USE_USBHS0
    rcu_usb48m_clock_config(IDX_USBHS0, RCU_USB48MSRC_IRC48M);
#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
    rcu_usb48m_clock_config(IDX_USBHS1, RCU_USB48MSRC_IRC48M);
#endif /* USE_USBHS1 */

#endif /* USE_IRC48M */

#endif /* USE_USB_FS */

#ifdef USE_USBHS0
    rcu_periph_clock_enable(RCU_USBHS0);
#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
    rcu_periph_clock_enable(RCU_USBHS1);
#endif /* USE_USBHS1 */

#ifdef USE_ULPI_PHY
#ifdef USE_USBHS0
    rcu_periph_clock_enable(RCU_USBHS0ULPI);
#endif

#ifdef USE_USBHS1
    rcu_periph_clock_enable(RCU_USBHS1ULPI);
#endif
#endif /* USE_ULPI_PHY */
}

/*!
    \brief      configure USB interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_intr_config(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);

#ifdef USE_USBHS0
    nvic_irq_enable(USBHS0_IRQn, 3U, 0U);
#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
    nvic_irq_enable(USBHS1_IRQn, 3U, 0U);
#endif /* USE_USBHS0 */

    /* enable the power module clock */
    rcu_periph_clock_enable(RCU_PMU);

#ifdef USE_USBHS0
    /* USB wakeup EXTI line configuration */
    exti_interrupt_flag_clear(EXTI_31);
    exti_init(EXTI_31, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_interrupt_enable(EXTI_31);

    nvic_irq_enable(USBHS0_WKUP_IRQn, 1U, 0U);
#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
    /* USB wakeup EXTI line configuration */
    exti_interrupt_flag_clear(EXTI_32);
    exti_init(EXTI_32, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_interrupt_enable(EXTI_32);

    nvic_irq_enable(USBHS1_WKUP_IRQn, 1U, 0U);
#endif /* USE_USBHS1 */
}

/*!
    \brief      drives the VBUS signal through GPIO
    \param[in]  state: VBUS states
    \param[out] none
    \retval     none
*/
void usb_vbus_drive(uint8_t state)
{
    if(0U == state) {
        /* disable is needed on output of the power switch */
        gpio_bit_reset(HOST_POWERSW_PORT, HOST_POWERSW_VBUS);
    } else {
        /* enable the power switch by driving the enable high */
        gpio_bit_set(HOST_POWERSW_PORT, HOST_POWERSW_VBUS);
    }
}

/*!
    \brief      configures the GPIO for the VBUS
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_vbus_config(void)
{
    rcu_periph_clock_enable(HOST_POWERSW_PORT_RCC);

    /* USBFS_VBUS_CTRL(PD13) GPIO pin configuration */
    gpio_mode_set(HOST_POWERSW_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, HOST_POWERSW_VBUS);
    gpio_output_options_set(HOST_POWERSW_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, HOST_POWERSW_VBUS);

    /* by default, disable is needed on output of the power switch */
    usb_vbus_drive(0U);

    /* delay is need for stabilizing the VBUS low in reset condition, 
     * when VBUS = 1 and reset-button is pressed by user 
     */
    usb_mdelay(200);
}

/*!
    \brief      initializes delay unit using Timer2
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_timer_init(void)
{
    /* configure the priority group to 2 bits */
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);

    /* enable the TIMER2 global interrupt */
    nvic_irq_enable(TIMER2_IRQn, 1U, 0U);

    rcu_periph_clock_enable(RCU_TIMER2);
}

/*!
    \brief      delay in microseconds
    \param[in]  usec: value of delay required in microseconds
    \param[out] none
    \retval     none
*/
void usb_udelay(const uint32_t usec)
{
    hw_delay(usec, TIM_USEC_DELAY);
}

/*!
    \brief      delay in milliseconds
    \param[in]  msec: value of delay required in milliseconds
    \param[out] none
    \retval     none
*/
void usb_mdelay(const uint32_t msec)
{
    hw_delay(msec, TIM_MSEC_DELAY);
}

/*!
    \brief      time base IRQ
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_timer_irq(void)
{
    if(RESET != timer_interrupt_flag_get(TIMER2, TIMER_INT_UP)) {
        timer_interrupt_flag_clear(TIMER2, TIMER_INT_UP);

        if(delay_time > 0x00U) {
            delay_time--;
        } else {
            timer_disable(TIMER2);
        }
    }
}

/*!
    \brief      delay routine based on TIMER2
    \param[in]  ntime: delay Time 
    \param[in]  unit: delay Time unit = miliseconds / microseconds
    \param[out] none
    \retval     none
*/
static void hw_delay(uint32_t ntime, uint8_t unit)
{
    delay_time = ntime;

    hw_time_set(unit);

    while(0U != delay_time) {
    }

    timer_disable(TIMER2);
}

/*!
    \brief      configures TIMER2 for delay routine based on TIMER2
    \param[in]  unit: msec /usec
    \param[out] none
    \retval     none
*/
static void hw_time_set(uint8_t unit)
{
    timer_parameter_struct timer_basestructure;

    timer_disable(TIMER2);
    timer_interrupt_disable(TIMER2, TIMER_INT_UP);

    if(TIM_USEC_DELAY == unit) {
        timer_basestructure.period = 9U;
    } else if(TIM_MSEC_DELAY == unit) {
        timer_basestructure.period = 9999U;
    } else {
        /* no operation */
    }

    timer_basestructure.prescaler         = timer_prescaler;
    timer_basestructure.alignedmode       = TIMER_COUNTER_EDGE;
    timer_basestructure.counterdirection  = TIMER_COUNTER_UP;
    timer_basestructure.clockdivision     = TIMER_CKDIV_DIV1;
    timer_basestructure.repetitioncounter = 0U;

    timer_init(TIMER2, &timer_basestructure);

    timer_interrupt_flag_clear(TIMER2, TIMER_INT_UP);

    timer_auto_reload_shadow_enable(TIMER2);

    /* TIMER IT enable */
    timer_interrupt_enable(TIMER2, TIMER_INT_UP);

    /* TIMER2 enable counter */ 
    timer_enable(TIMER2);
}

/*!
    \brief      configure the PLL of USB
    \param[in]  usb_periph: USBHS0 or USBHS1
    \param[out] none
    \retval     none
*/
void pllusb_rcu_config(uint32_t usb_periph)
{
    if(USBHS0 == usb_periph) {
        rcu_pllusb0_config(RCU_PLLUSBHSPRE_HXTAL, RCU_PLLUSBHSPRE_DIV5, RCU_PLLUSBHS_MUL96, RCU_USBHS_DIV10);
        RCU_ADDCTL1 |= RCU_ADDCTL1_PLLUSBHS0EN;
        while(0U == (RCU_ADDCTL1 & RCU_ADDCTL1_PLLUSBHS0STB)) {
        }

        rcu_usb48m_clock_config(IDX_USBHS0, RCU_USB48MSRC_PLLUSBHS);
    } else {
        rcu_pllusb1_config(RCU_PLLUSBHSPRE_HXTAL, RCU_PLLUSBHSPRE_DIV5, RCU_PLLUSBHS_MUL96, RCU_USBHS_DIV10);
        RCU_ADDCTL1 |= RCU_ADDCTL1_PLLUSBHS1EN;
        while(0U == (RCU_ADDCTL1 & RCU_ADDCTL1_PLLUSBHS1STB)) {
        }

        rcu_usb48m_clock_config(IDX_USBHS1, RCU_USB48MSRC_PLLUSBHS);
    }
}

