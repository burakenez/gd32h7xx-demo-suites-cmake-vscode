/*!
    \file    bsp_ts_gt911.h
    \brief   the header file of gt911 driver

    \version 2023-03-31, V1.0.0, demo for GD32H7xx
*/

/*
    Copyright (c) 2023, GigaDevice Semiconductor Inc.

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

#ifndef BSP_GT911_H
#define BSP_GT911_H

#include "gd32h7xx.h"
#include "gd32h7xx.h"

/* constants for touch point and refresh rate */
#define TOUCH_POINT           (uint8_t)0x01            /* number of touch points (1 to 5) */
#define TOUCH_REFRESH         (uint8_t)10              /* refresh rate (5 + N ms, min=13ms, valid values: 15ms, 20ms) */

/* gt911 register addresses */
#define GT911_PRESSED_INFO_REG 0x814E                  /* address of pressed information register */
#define GT911_COORDINATE_REG   0x814F                  /* address of coordinate register */
#define GT911_CFG_START_ADDR   0x8047                  /* start address of configuration */
#define GT911_CFG_NUM          (0x80FE - 0x8047 + 1)   /* number of configuration parameters */

/* structure to store gt911 touch data */
typedef struct {
    uint8_t enable;                                    /* enable flag for gt911 */
    uint8_t pressed;                                   /* press status (0 - unpressed, non-zero - pressed) */
    uint8_t pressed_info;                              /* pressed information (register 0x814E) */
    uint16_t x[5];                                     /* X coordinates of touch points */
    uint16_t y[5];                                     /* Y coordinates of touch points */
    uint16_t s[5];                                     /* sizes of touch points */
} gt911_struct;

/* external variable for storing gt911 touch data */
extern gt911_struct gt911;

/* initializes the gt911 touch controller */
ErrStatus gt911_init(void);
/* reads the gt911 device ID */
uint32_t gt911_read_id(void);
/* reads the gt911 firmware version */
uint16_t gt911_read_version(void);
/* reads the gt911 screen resolution */
uint32_t gt911_read_resolution(void);
/* handles gt911 touch events and updates the touch state */
ErrStatus gt911_handler(void);

#endif /* BSP_GT911_H */
