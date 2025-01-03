/*!
    \file    gd32h7xx_it.c
    \brief   main interrupt service routines

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

#include "gd32h7xx_it.h"
#include "drv_usb_hw.h"
#include "drv_usbh_int.h"

extern usbh_host usb_host_hid;
extern usb_core_driver hid_host_core;

extern void usb_timer_irq(void);

/* local function prototypes ('static') */
static void resume_mcu_clk(void);

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
    /* if NMI exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
    /* if DebugMon exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
    /* if SVC exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
    /* if PendSV exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles FPU exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void FPU_IRQHandler(void)
{
    while(1) { 
    }
}

/*!
    \brief      this function handles Timer2 update interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void TIMER2_IRQHandler(void)
{
    usb_timer_irq();
}

#ifdef USE_USBHS0
/*!
    \brief      this function handles USBHS0 interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USBHS0_IRQHandler(void)
{
    usbh_isr(&hid_host_core);
}

#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
/*!
    \brief      this function handles USBHS1 interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USBHS1_IRQHandler(void)
{
    usbh_isr(&hid_host_core);
}

#endif /* USE_USBHS1 */

#if USB_LOW_POWER

/*!
    \brief      this function handles external line 0 interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void EXTI0_IRQHandler(void)
{
    if(RESET != exti_interrupt_flag_get(WAKEUP_KEY_EXTI_LINE)) {
        if(1U == usb_host_hid.suspend_flag) {
            usb_host_hid.suspend_flag = 0U;
            usb_host_hid.wakeup_mode = GENERAL_WAKEUP; /* general wakeup mode */

            /* configure system clock */
            resume_mcu_clk();

            usb_rcu_config();

#ifdef USE_USB_HS
    #ifdef USE_USBHS0
        pllusb_rcu_config(USBHS0);
    #elif defined USE_USBHS1
        pllusb_rcu_config(USBHS1);
    #else
    #endif
#endif /* USE_USB_HS */
        }

        /* clear the EXTI line pending bit */
        exti_interrupt_flag_clear(WAKEUP_KEY_EXTI_LINE);
    }
}

#ifdef USE_USBHS0
/*!
    \brief      this function handles USBHS0 wakeup interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USBHS0_WKUP_IRQHandler(void)
{
    if(1U == usb_host_hid.suspend_flag) {
        usb_host_hid.suspend_flag = 0U;

        /* configure system clock */
        resume_mcu_clk();

        usb_rcu_config();

        usb_host_hid.wakeup_mode = REMOTE_WAKEUP; /* remote wakeup mode */

#ifdef USE_USB_HS
        /* configure usb clock */
        pllusb_rcu_config(USBHS0);
#endif /* USE_USB_HS */
    }

    exti_interrupt_flag_clear(EXTI_31);
}

#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
/*!
    \brief      this function handles USBHS1 wakeup interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USBHS1_WKUP_IRQHandler(void)
{
    if(1U == usb_host_hid.suspend_flag) {
        usb_host_hid.suspend_flag = 0U;
        usb_host_hid.wakeup_mode = REMOTE_WAKEUP; /* remote wakeup mode */

        /* configure system clock */
        resume_mcu_clk();

        usb_rcu_config();

#ifdef USE_USB_HS
        /* configure usb clock */
        pllusb_rcu_config(USBHS1);
#endif /* USE_USB_HS */
    }

    exti_interrupt_flag_clear(EXTI_32);
}

#endif /* USE_USBHS1 */

#endif /* USB_LOW_POWER */

/*!
    \brief      resume MCU clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void resume_mcu_clk(void)
{
    /* enable HXTAL */
    rcu_osci_on(RCU_HXTAL);

    /* wait till HXTAL is ready */
    while(RESET == rcu_flag_get(RCU_FLAG_HXTALSTB)) {
    }

    /* enable PLL */
    rcu_osci_on(RCU_PLL0_CK);

    /* wait till PLL is ready */
    while(RESET == rcu_flag_get(RCU_FLAG_PLL0STB)) {
    }

    /* select PLL as system clock source */
    rcu_system_clock_source_config(RCU_CKSYSSRC_PLL0P);

    /* wait till PLL is used as system clock source */
    while(RCU_SCSS_PLL0P != rcu_system_clock_source_get()) {
    }
}
