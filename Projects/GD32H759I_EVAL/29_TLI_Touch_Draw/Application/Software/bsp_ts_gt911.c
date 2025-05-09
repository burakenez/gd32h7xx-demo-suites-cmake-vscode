/*!
    \file    gd32h7xx_it.c
    \brief   interrupt service routines

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

#include "bsp_ts_gt911.h"
#include "bsp_i2c_touch.h"
#include "string.h"
#include "systick.h"
/* configuration parameters array for gt911, to be written to gt911 in one go */
const uint8_t gt911_cfg_params[] = {
0x5A,0xE0,0x01,0x10,0x01,0x01,0x8D,0x00,0x01,0x08,0x28,0x05,
0x55,0x32,0x03,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x89,0x2A,0x0B,0x17,0x15,0x31,0x0D,0x00,0x00,
0x02,0xB9,0x04,0x2C,0x00,0x00,0x00,0x00,0x00,0x03,0x64,0x32,
0x00,0x00,0x00,0x0F,0x94,0x94,0xC5,0x02,0x07,0x00,0x00,0x04,
0x8D,0x13,0x00,0x5C,0x1E,0x00,0x3C,0x30,0x00,0x29,0x4C,0x00,
0x20,0x78,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x10,
0x12,0x14,0x16,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x24,0x22,
0x21,0x20,0x1F,0x1E,0x1D,0x1C,0x18,0x16,0x12,0x10,0x0F,0x0A,
0x08,0x06,0x04,0x02,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0x21,0x01


};

/* structure for storing gt911 data */
gt911_struct gt911 = {0};                                   /* initialize gt911 data structure to zero */
uint8_t clear_flag = 0;                                     /* initialize clear flag to zero */ 
uint8_t cfg_buf[sizeof(gt911_cfg_params)] = {0};            /* configuration buffer */
uint8_t touch_buf[48];                                      /* touch buffer */


/*!
    \brief      initializes the gt911 touch controller.
    \param[in]  none
    \param[out] none
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus gt911_init(void)
{
    ErrStatus err;
    uint8_t i; 
    uint8_t checksum;
    /* number of configuration parameters */
    uint8_t cfg_num = GT911_CFG_NUM;  

    /* clear gt911 data structure */
    memset(&gt911, 0, sizeof(gt911));

    /* initialize i2c bus and gt911 */
    i2c_touch_init();
    
    /* calculate checksum */
    memcpy(cfg_buf, gt911_cfg_params, cfg_num);
    checksum = 0;
    for (i = 0; i < cfg_num; i++) {
        checksum += cfg_buf[i];
    }
    cfg_buf[sizeof(gt911_cfg_params) - 2] = (~checksum) + 1;
    cfg_buf[sizeof(gt911_cfg_params) - 1] = 1;
    
    /* wait for gt911 initialization */
    delay_ms(100); 

    /* configure gt911 */
    if (ERROR != gt911_writereg(GT911_CFG_START_ADDR, (uint8_t *)cfg_buf, sizeof(gt911_cfg_params))) {
        /* wait for configuration to be valid */
        delay_ms(10); 
        gt911.enable = 1;
        err = SUCCESS;
    } else {
        gt911.enable = 0;
        err = ERROR;
    }  
    return err;
}


/*!
    \brief      gt911 handler using polling method.
    \param[in]  none
    \param[out] none
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus gt911_handler(void) {
    ErrStatus err = SUCCESS; 
    /* return error if gt911 is not enabled */
    if (gt911.enable == 0) {
        return err; 
    }
    /* read gt911 pressed info */
    err &= gt911_readreg(GT911_PRESSED_INFO_REG, touch_buf, 1);
    gt911.pressed_info = touch_buf[0];
    
    if (gt911.pressed_info & 0x0f) {
        /* read coordinate data */
        err &= gt911_readreg(GT911_COORDINATE_REG, touch_buf, 8 * (gt911.pressed_info & 0x0f));
        /* store coordinate data */
        for (uint8_t i = 0; i < (gt911.pressed_info & 0x0f); i++) {
            gt911.x[i] = ((uint16_t)touch_buf[2 + i * 8] << 8) + touch_buf[1 + i * 8];
            gt911.y[i] = ((uint16_t)touch_buf[4 + i * 8] << 8) + touch_buf[3 + i * 8];
            gt911.s[i] = ((uint16_t)touch_buf[6 + i * 8] << 8) + touch_buf[5 + i * 8];
        }
        /* touch detected, set pressed to 2 (release time) */
        gt911.pressed = 2;
    } else if (gt911.pressed) {
        /* no new touch detected, check for release */ 
        gt911.pressed--;
    }
    
    if (gt911.pressed_info) {
        /* clear pressed info */
        err &= gt911_writereg(GT911_PRESSED_INFO_REG, &clear_flag, 1);
    }
    
    return err;
}


/*!
    \brief      reads the ID of the gt911 touch controller.
    \param[in]  none
    \param[out] none
    \retval     uint32_t: ID of the gt911
*/
uint32_t gt911_read_id(void)
{
    uint8_t id_buf[4];

    if(SUCCESS != gt911_readreg(0x8140, id_buf, 4)) {
        return 0;
    } else {
        return ((uint32_t)id_buf[3] << 24) + ((uint32_t)id_buf[2] << 16) + ((uint32_t)id_buf[1] << 8) + id_buf[0];
    }
}

/*!
    \brief      reads the version of the gt911 firmware.
    \param[in]  none
    \param[out] none
    \retval     uint16_t: Firmware version of the gt911
*/
uint16_t gt911_read_version(void)
{
    uint8_t tmp_buf[2];

    if(SUCCESS != gt911_readreg(0x8144, tmp_buf, 2)) {
        return 0;
    } else {
        return ((uint16_t)tmp_buf[1] << 8) + tmp_buf[0];
    }
}

/*!
    \brief      reads the resolution of the gt911 touch controller.
    \param[in]  none
    \param[out] none
    \retval     uint32_t: Resolution of the gt911
*/
uint32_t gt911_read_resolution(void)
{
    uint8_t tmp_buf[4];
    if(SUCCESS != gt911_readreg(0x8146, tmp_buf, 4)) {
        return 0;
    } else {
        return (((uint32_t)tmp_buf[3] << 24) | ((uint32_t)tmp_buf[2] << 16) | ((uint32_t)tmp_buf[1] << 8) | ((uint32_t)tmp_buf[0]));
    }
}

/*!
    \brief      This function handles GTP_IRQHandler exception.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void GTP_IRQHandler(void)
{
    if(exti_interrupt_flag_get(GTP_INT_EXTI_LINE) != RESET) {
        exti_interrupt_flag_clear(GTP_INT_EXTI_LINE);
        /* get gt911 data using interrupt */
        gt911_handler();
    }  
}
