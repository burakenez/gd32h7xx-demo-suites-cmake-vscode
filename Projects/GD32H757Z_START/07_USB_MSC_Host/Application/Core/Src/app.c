/*!
    \file    app.c
    \brief   main routine

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

#include "drv_usb_hw.h"
#include "usbh_msc_core.h"
#include "usbh_usr.h"

usbh_host usb_host_msc;
usb_core_driver msc_host_core;

/*!
    \brief      enable the CPU cache
    \param[in]  none
    \param[out] none
    \retval     none
*/
void cache_enable(void)
{
    /* enable i-cache */
    SCB_EnableICache();

    /* enable d-cache */
    /** note:
      * if the USB DMA is enabled, the d-cache should be disabled!
      */
#ifndef USB_INTERNAL_DMA_ENABLED
    SCB_EnableDCache();
#endif /* USB_INTERNAL_DMA_ENABLED */
}

/*!
    \brief      main routine will construct a USB host
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    cache_enable();

    usb_rcu_config();

    usb_timer_init();

    /* configure GPIO pin used for switching VBUS power and charge pump I/O */
    usb_vbus_config();

    /* register device class */
    usbh_class_register(&usb_host_msc, &usbh_msc);

#ifdef USE_USB_FS
    usb_para_init(&msc_host_core, USBHS0, USB_SPEED_FULL);
#endif

#ifdef USE_USB_HS
    usb_para_init(&msc_host_core, USBHS0, USB_SPEED_HIGH);
#endif

    usbh_init(&usb_host_msc, &msc_host_core, &usr_cb);

#ifdef USE_USB_HS
    pllusb_rcu_config(USBHS0);
#endif /* USE_USB_HS */

    usb_intr_config();

    while(1) {
        usbh_core_task(&usb_host_msc);
    }
}
