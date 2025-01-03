/*!
    \file    app.c
    \brief   main routine

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
#include "standard_hid_core.h"

extern hid_fop_handler fop_handler;

usb_core_driver hid_keyboard;

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
    \brief      main routine will construct a USB HID keyboard device
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    cache_enable();

    usb_rcu_config();

    usb_timer_init();

    hid_itfop_register(&hid_keyboard, &fop_handler);

#ifdef USE_USBHS0

#ifdef USE_USB_FS
    usb_para_init(&hid_keyboard, USBHS0, USB_SPEED_FULL);
#endif

#ifdef USE_USB_HS
    usb_para_init(&hid_keyboard, USBHS0, USB_SPEED_HIGH);
#endif

#endif /* USE_USBHS0 */

#ifdef USE_USBHS1

#ifdef USE_USB_FS
    usb_para_init(&hid_keyboard, USBHS1, USB_SPEED_FULL);
#endif

#ifdef USE_USB_HS
    usb_para_init(&hid_keyboard, USBHS1, USB_SPEED_HIGH);
#endif

#endif /* USE_USBHS1 */

    usbd_init(&hid_keyboard, &hid_desc, &usbd_hid_cb);

#ifdef USE_USB_HS
    #ifndef USE_ULPI_PHY
        #ifdef USE_USBHS0
            pllusb_rcu_config(USBHS0);
        #elif defined USE_USBHS1
            pllusb_rcu_config(USBHS1);
        #else
        #endif
    #endif /* !USE_ULPI_PHY */
#endif /* USE_USB_HS */

    usb_intr_config();

    /* check if USB device is enumerated successfully */
    while(USBD_CONFIGURED != hid_keyboard.dev.cur_status) {
    }

    while(1) {
        fop_handler.hid_itf_data_process(&hid_keyboard);
    }
}
