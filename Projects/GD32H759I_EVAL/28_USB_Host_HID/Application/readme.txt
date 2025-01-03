/*!
    \file    readme.txt
    \brief   description of the USB keyboard demo

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

  This demo is based on the GD32H759I-EVAL-V2.0 board, it provides a description of 
how to use the USB_HS1_Host peripheral. After connecting the cable from USB to serial port, 
the related information can print out on the serial port assistant.

  When an USB Device is attached to the Host port, the device is enumerated and checked
whether it can support HID device or not, if the attached device is HID one, when the user
press the User key, the mouse or the keyboard application is launched.

  Moving the mouse will move the pointer in the display rectangle and if a button is pressed,
the corresponding rectangle will be highlighted in green.

  If a keyboard has been attached, the display show the following messages and the taped
characters are displayed in green on the display.

  In the USB Host HID class, two layouts are defined in the usbh_hid_keybd.h file and could 
be used (Azerty and Querty).

  //#define QWERTY_KEYBOARD
  #define AZERTY_KEYBOARD

  The User can eventually add his own layout by editing the HID_KEYBRD_Key array in the 
usbh_hid_keybd.c file.

  The demo support the functions of host suspend and wakup. The macro of USB_LOW_POWER can 
be set to 1 to test the suspend and wakeup. If you want to use the general wakeup mode, please 
press the Wakup key. If you want to use the remote wakeup mode, please operating device, such as 
move the mouse or press the keyboard. If you want the program to continue running, please press 
the Wakeup key.

    JP70 must be fit to USB.
    JP68 must be fit to USART.
