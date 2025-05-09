/*!
    \file    bsp_i2c_touch.h
    \brief   the header file of i2c touch driver

    \version 2025-02-19, V2.1.0, demo for GD32H7xx
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

#ifndef I2C_TOUCH_H
#define I2C_TOUCH_H

#include "gd32h7xx.h"

/* IIC address*/
#define GTP_ADDRESS                      0xBA

/* i2c_scl */
#define GTP_I2C_SCL_PIN                  GPIO_PIN_7
#define GTP_I2C_SCL_GPIO_PORT            GPIOH
#define GTP_I2C_SCL_GPIO_CLK             RCU_GPIOH
#define GTP_I2C_SCL_AF                   GPIO_AF_4
/* i2c_sda */
#define GTP_I2C_SDA_PIN                  GPIO_PIN_9
#define GTP_I2C_SDA_GPIO_PORT            GPIOF
#define GTP_I2C_SDA_GPIO_CLK             RCU_GPIOF
#define GTP_I2C_SDA_AF                   GPIO_AF_4

/* RST_GPIO */
#define GTP_RST_GPIO_CLK                 RCU_GPIOF
#define GTP_RST_GPIO_PORT                GPIOF
#define GTP_RST_GPIO_PIN                 GPIO_PIN_6
#define GTP_RST_1()                      gpio_bit_set(GTP_RST_GPIO_PORT,GTP_RST_GPIO_PIN)
#define GTP_RST_0()                      gpio_bit_reset(GTP_RST_GPIO_PORT,GTP_RST_GPIO_PIN)

/* INT_GPIO */
#define GTP_INT_GPIO_CLK                 RCU_GPIOH
#define GTP_INT_GPIO_PORT                GPIOH
#define GTP_INT_GPIO_PIN                 GPIO_PIN_6
#define GTP_INT_1()                      gpio_bit_set(GTP_INT_GPIO_PORT,GTP_INT_GPIO_PIN)
#define GTP_INT_0()                      gpio_bit_reset(GTP_INT_GPIO_PORT,GTP_INT_GPIO_PIN)
#define GTP_INT_GET()                    gpio_input_bit_get(GTP_INT_GPIO_PORT,GTP_INT_GPIO_PIN)
/* INT_EXTI */
#define GTP_INT_EXTI_PORTSOURCE          EXTI_SOURCE_GPIOA
#define GTP_INT_EXTI_PINSOURCE           EXTI_SOURCE_PIN5
#define GTP_INT_EXTI_LINE                EXTI_5
#define GTP_INT_EXTI_IRQ                 EXTI5_9_IRQn
#define GTP_IRQHandler                   EXTI5_9_IRQHandler

#define I2C_SCL_1()                      gpio_bit_set(GTP_I2C_SCL_GPIO_PORT, GTP_I2C_SCL_PIN)          /* set SCL to 1 */
#define I2C_SCL_0()                      gpio_bit_reset(GTP_I2C_SCL_GPIO_PORT, GTP_I2C_SCL_PIN)        /* set SCL to 0 */
#define I2C_SDA_1()                      gpio_bit_set(GTP_I2C_SDA_GPIO_PORT, GTP_I2C_SDA_PIN)          /* set SDA to 1 */
#define I2C_SDA_0()                      gpio_bit_reset(GTP_I2C_SDA_GPIO_PORT, GTP_I2C_SDA_PIN)        /* set SDA to 0 */
#define I2C_SDA_READ()                   gpio_input_bit_get(GTP_I2C_SDA_GPIO_PORT, GTP_I2C_SDA_PIN)    /* read SDA line status */

#define I2C_DIR_WR    0        /* write control bit */
#define I2C_DIR_RD    1        /* read control bit */

/* reset the gt911 touch controller */
void i2c_resetchip(void);
/* enables the interrupt for gt911 touch controller */
void i2c_gtp_irqenable(void);
/* initializes the I2C touch controller */
void i2c_touch_init(void);
/* reads a sequence of bytes from an I2C device */
ErrStatus i2c_readbytes(uint8_t addr, uint8_t *buffer, uint16_t number);
/* writes a sequence of bytes to an I2C device */
ErrStatus i2c_writebytes(uint8_t addr, uint8_t *buffer, uint8_t number);
/* reads a sequence of bytes from the gt911 register */
ErrStatus gt911_readreg(uint16_t addr, uint8_t *buffer, uint8_t length);
/* writes a sequence of bytes to the gt911 register. */
ErrStatus gt911_writereg(uint16_t addr, uint8_t *buffer, uint8_t length);
#endif /* I2C_TOUCH_H */
