/*!
    \file    gd32h7xx_usb_hw.c
    \brief   USB hardware configuration for GD32H7xx

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

#include "drv_usb_hw.h"

#define TIM_MSEC_DELAY                          0x01U
#define TIM_USEC_DELAY                          0x02U

__IO uint32_t delay_time = 0U;
__IO uint16_t timer_prescaler = 23U;

/* local function prototypes ('static') */
static void hw_time_set(uint8_t unit);
static void hw_delay(uint32_t ntime, uint8_t unit);

/*!
    \brief      configure USB GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_gpio_config (void)
{
#ifdef USE_USBHS1 
    rcu_periph_clock_enable(RCU_GPIOJ);
    rcu_periph_clock_enable(RCU_GPIOG);
    rcu_periph_clock_enable(RCU_GPIOK);
    
    /* ULPI_STP(PJ8) GPIO pin configuration */
    gpio_mode_set(GPIOJ, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_output_options_set(GPIOJ, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_8);

    /* ULPI_CK(PJ11) GPIO pin configuration */
    gpio_mode_set(GPIOJ, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_output_options_set(GPIOJ, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_11);

    /* ULPI_NXT(PJ10) GPIO pin configuration */
    gpio_mode_set(GPIOJ, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_10);
    gpio_output_options_set(GPIOJ, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_10);

    /* ULPI_DIR(PJ9) GPIO pin configuration */
    gpio_mode_set(GPIOJ, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9);
    gpio_output_options_set(GPIOJ, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_9);

    /* ULPI_D0(PK0) GPIO pin configuration */
    /* ULPI_D1(PK1), ULPI_D2(PK2), ULPI_D3(PG1), ULPI_D4(PG2) \
       ULPI_D5(PG3), ULPI_D6(PG4) and ULPI_D7(PG5) GPIO pin configuration */
    gpio_mode_set(GPIOK, GPIO_MODE_AF, GPIO_PUPD_NONE, \
                    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
    gpio_output_options_set(GPIOK, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, \
                    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);

    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, \
                    GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, \
                    GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

    gpio_af_set(GPIOJ, GPIO_AF_7, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11);
    gpio_af_set(GPIOK, GPIO_AF_7, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
    gpio_af_set(GPIOG, GPIO_AF_8, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
#endif /* USE_USBHS1 */
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

#ifdef USE_USB_FS

#ifndef USE_IRC48M

#ifdef USE_USBHS0
    /* GUSBCS EMBPHY_FS enabled */
    *(uint32_t*)0x4004000C |= 0x40;
    /* GCCFG power on */
    *(uint32_t*)0x40040038 |= 0x10000;

    rcu_pllusb0_config(RCU_PLLUSBHSPRE_HXTAL, RCU_PLLUSBHSPRE_DIV5, RCU_PLLUSBHS_MUL96, RCU_USBHS_DIV10);
    RCU_ADDCTL1 |= RCU_ADDCTL1_PLLUSBHS0EN;
    while(0U == (RCU_ADDCTL1 & RCU_ADDCTL1_PLLUSBHS0STB)) {
    }

    rcu_usb48m_clock_config(IDX_USBHS0, RCU_USB48MSRC_PLLUSBHS);
#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
    /* GUSBCS EMBPHY_FS enabled */
    *(uint32_t*)0x4008000C |= 0x40;
    /* GCCFG power on */
    *(uint32_t*)0x40080038 |= 0x10000;

    rcu_pllusb1_config(RCU_PLLUSBHSPRE_HXTAL, RCU_PLLUSBHSPRE_DIV5, RCU_PLLUSBHS_MUL96, RCU_USBHS_DIV10);
    RCU_ADDCTL1 |= RCU_ADDCTL1_PLLUSBHS1EN;
    while(0U == (RCU_ADDCTL1 & RCU_ADDCTL1_PLLUSBHS1STB)) {
    }

    rcu_usb48m_clock_config(IDX_USBHS1, RCU_USB48MSRC_PLLUSBHS);

#endif /* USE_USBHS0 */

#else
    /* enable IRC48M clock */
    rcu_osci_on(RCU_IRC48M);

    /* wait till IRC48M is ready */
    while(SUCCESS != rcu_osci_stab_wait(RCU_IRC48M)) {
    }

#ifdef USE_USBHS0
    rcu_usb48m_clock_config(IDX_USBHS0, RCU_USB48MSRC_IRC48M);
#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
    rcu_usb48m_clock_config(IDX_USBHS1, RCU_USB48MSRC_IRC48M);
#endif /* USE_USBHS1 */

#endif /* USE_IRC48M */

#endif /* USE_USB_FS */
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
    nvic_irq_enable((uint8_t)USBHS0_IRQn, 3U, 0U);
#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
    nvic_irq_enable((uint8_t)USBHS1_IRQn, 3U, 0U);
#endif /* USE_USBHS0 */

    /* enable the power module clock */
    rcu_periph_clock_enable(RCU_PMU);

#ifdef USE_USBHS0
    /* USB wakeup EXTI line configuration */
    exti_interrupt_flag_clear(EXTI_31);
    exti_init(EXTI_31, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_interrupt_enable(EXTI_31);

    nvic_irq_enable((uint8_t)USBHS0_WKUP_IRQn, 1U, 0U);
#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
    /* USB wakeup EXTI line configuration */
    exti_interrupt_flag_clear(EXTI_32);
    exti_init(EXTI_32, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_interrupt_enable(EXTI_32);

    nvic_irq_enable((uint8_t)USBHS1_WKUP_IRQn, 1U, 0U);
#endif /* USE_USBHS1 */

#ifdef USB_DEDICATED_EP1_ENABLED

#ifdef USE_USBHS0
    nvic_irq_enable((uint8_t)USBHS0_EP1_OUT_IRQn, 1U, 0U);
    nvic_irq_enable((uint8_t)USBHS0_EP1_IN_IRQn, 1U, 0U);
#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
    nvic_irq_enable((uint8_t)USBHS1_EP1_OUT_IRQn, 1U, 0U);
    nvic_irq_enable((uint8_t)USBHS1_EP1_IN_IRQn, 1U, 0U);
#endif /* USE_USBHS1 */

#endif /* USB_DEDICATED_EP1_ENABLED */
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

    /* enable the TIM2 global interrupt */
    nvic_irq_enable((uint8_t)TIMER2_IRQn, 1U, 0U);

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
        rcu_pllusb0_config(RCU_PLLUSBHSPRE_HXTAL, RCU_PLLUSBHSPRE_DIV5, RCU_PLLUSBHS_MUL96, RCU_USBHS_DIV8);
        RCU_ADDCTL1 |= RCU_ADDCTL1_PLLUSBHS0EN;
        while(0U == (RCU_ADDCTL1 & RCU_ADDCTL1_PLLUSBHS0STB)) {
        }
    } else {
        rcu_pllusb1_config(RCU_PLLUSBHSPRE_HXTAL, RCU_PLLUSBHSPRE_DIV5, RCU_PLLUSBHS_MUL96, RCU_USBHS_DIV8);
        RCU_ADDCTL1 |= RCU_ADDCTL1_PLLUSBHS1EN;
        while(0U == (RCU_ADDCTL1 & RCU_ADDCTL1_PLLUSBHS1STB)) {
        }
    }
}
