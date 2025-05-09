/*!
    \file    bsp_i2c_touch.c
    \brief   bsp_i2c_touch driver

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

#include "bsp_i2c_touch.h"
#include "systick.h"
#include <stdio.h>
#include <string.h>

/* generates an I2C bus delay */
static void i2c_delay(void);
/* generates an I2C start signal */
static void i2c_start(void);
/* generates an I2C stop signal */
static void i2c_stop(void);
/* sends a byte via I2C bus */
static void i2c_sendbyte(uint8_t byte);
/* reads a byte from the I2C bus */
static uint8_t i2c_readbyte(void);
/* waits for ACK signal from the I2C device */
static uint8_t i2c_waitack(void);
/* generates an ACK signal on the I2C bus */
static void i2c_ack(void);
/* generates a NACK signal on the I2C bus */
static void i2c_nack(void);
/* configures the GPIO pins for I2C communication */
static void i2c_gpio_config(void);

/*!
    \brief      reset the gt911 touch controller
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c_resetchip(void)
{
    /* initialize gt911, RST high, INT low, sets device address to 0xBA */
    GTP_RST_0();
    GTP_INT_0();
    delay_ms(1);

    /* init low >5ms */
    GTP_RST_1();
    delay_ms(6);

    /* configure INT as input */
    gpio_mode_set(GTP_INT_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GTP_INT_GPIO_PIN);
}

/*!
    \brief      enables the interrupt for gt911 touch controller
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c_gtp_irqenable(void)
{
    rcu_periph_clock_enable(GTP_INT_GPIO_CLK);
    rcu_periph_clock_enable(RCU_SYSCFG);

    /* enable and set EXTI interrupt priority */
    nvic_irq_enable(GTP_INT_EXTI_IRQ, 14, 0U);

    /* connect EXTI line to GPIO pin */
    syscfg_exti_line_config(GTP_INT_EXTI_PORTSOURCE, GTP_INT_EXTI_PINSOURCE);

    /* configure EXTI line */
    exti_init(GTP_INT_EXTI_LINE, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_interrupt_flag_clear(GTP_INT_EXTI_LINE);
}

/*!
    \brief      initializes the I2C touch controller
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c_touch_init(void)
{
    /* configure GPIO pins for I2C */
    i2c_gpio_config();

    /* reset the touch controller */
    i2c_resetchip();
}

/*!
    \brief      reads a sequence of bytes from an I2C device.
    \param[in]  addr: the I2C device address to read from.
                buffer: pointer to the data buffer where the read data will be stored.
                number: number of bytes to read.
    \param[out] none
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus i2c_readbytes(uint8_t addr, uint8_t *buffer, uint16_t number)
{
    /* step 1: send I2C start signal */
    i2c_start();

    /* step 2: send control byte with address and read signal */
    i2c_sendbyte(addr | I2C_DIR_RD);    

    /* step 3: wait for ACK */
    if(i2c_waitack() != 0) {
        /* send I2C stop signal in case of failure */
        i2c_stop();
        /* no response from device */
        return ERROR;
    }

    while(number) {
        if(number == 1) {
            /* send NACK after the last byte */
            i2c_nack();

            /* send I2C stop signal */
            i2c_stop();
        }

        /* read data byte */
        *buffer = i2c_readbyte();

        /* increment buffer pointer */
        buffer++;

        /* decrement counter */
        number--;

        if (number > 0) {
            /* send ACK after reading a byte */
            i2c_ack();
        }
    }

    /* send I2C stop signal */
    i2c_stop();
    /* operation successful */
    return SUCCESS;
}

/*!
    \brief      writes a sequence of bytes to an I2C device.
    \param[in]  addr: the I2C device address to write to.
                buffer: pointer to the data to be written.
                number: number of bytes to write.
    \param[out] none
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus i2c_writebytes(uint8_t addr, uint8_t *buffer, uint8_t number)
{
    __IO uint16_t m;
    /* default to failure */
    ErrStatus result = ERROR;

    /* step 0: send stop signal to initiate internal write operation */
    i2c_stop();

    /* check for device response, typically takes less than 10ms
       with a CLK frequency of 200KHz, about 30 iterations */
    for(m = 0; m < 1000; m++) {
        /* step 1: send I2C start signal */
        i2c_start();

        /* step 2: send control byte with address and write signal */
        i2c_sendbyte(addr | I2C_DIR_WR);

        /* step 3: wait for device ACK */
        if(i2c_waitack() == 0) {
            result = SUCCESS;    /* device responded, proceed with write */
            break;
        }
    }

    if(result == SUCCESS) {
        while(number--) {
            /* step 4: write data byte */
            i2c_sendbyte(*buffer);

            /* step 5: check for ACK */
            if(i2c_waitack() != SUCCESS) {
            /* no response from device */
                result = ERROR;
                break;
            }

            /* increment buffer pointer */
            buffer++;
        }
    }

    /* send I2C stop signal */
    i2c_stop();

    return result;
}


/*!
    \brief      reads a sequence of bytes from the gt911 register.
    \param[in]  addr: the register address to read from.
                buffer: pointer to the data buffer where the read data will be stored.
                length: number of bytes to read.
    \param[out] none
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus gt911_readreg(uint16_t addr, uint8_t *buffer, uint8_t length)
{
    __IO uint8_t i;

    /* start signal */
    i2c_start();

    /* send device address with write signal */
    i2c_sendbyte(GTP_ADDRESS | I2C_DIR_WR);
    if (i2c_waitack() != 0) {
        i2c_stop();
        return ERROR;
    }

    /* send high byte of the register address */
    i2c_sendbyte(addr >> 8);
    if (i2c_waitack() != 0) {
        i2c_stop();
        return ERROR;
    }

    /* send low byte of the register address */
    i2c_sendbyte(addr);
    if (i2c_waitack() != 0) {
        i2c_stop();
        return ERROR;
    }

    /* stop signal */
    i2c_stop();

    /* start signal */
    i2c_start();

    /* send device address with read signal */
    i2c_sendbyte(GTP_ADDRESS | I2C_DIR_RD);  
    if (i2c_waitack() != 0) {
        i2c_stop();
        return ERROR;
    }

    for (i = 0; i < length - 1; i++) {
        /* read register data */
        buffer[i] = i2c_readbyte();
        i2c_ack();
    }

    /* read last byte of register data */
    buffer[i] = i2c_readbyte();
    i2c_nack();

    /* stop signal */
    i2c_stop();

    return SUCCESS;
}



/*!
    \brief      writes a sequence of bytes to the gt911 register.
    \param[in]  addr: the register address to write to.
                buffer: pointer to the data to be written.
                length: number of bytes to write.
    \param[out] none
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus gt911_writereg(uint16_t addr, uint8_t *buffer, uint8_t length)
{
    __IO uint8_t i;

    /* start signal */
    i2c_start();

    /* send device address with write signal */
    i2c_sendbyte(GTP_ADDRESS | I2C_DIR_WR);
    if (i2c_waitack() != 0) {
        i2c_stop();
        return ERROR;
    }

    /* send high byte of the register address */
    i2c_sendbyte(addr >> 8);
    if (i2c_waitack() != 0) {
        i2c_stop();
        return ERROR;
    }

    /* send low byte of the register address */
    i2c_sendbyte(addr);
    if (i2c_waitack() != 0) {
        i2c_stop();
        return ERROR;
    }

    for (i = 0; i < length; i++) {
        /* send the data bytes */
        i2c_sendbyte(buffer[i]);
        if (i2c_waitack() != 0) {
            i2c_stop();
            return ERROR;
        }
    }

    /* stop signal */
    i2c_stop(); 
    return SUCCESS;
}

/*!
    \brief      generates an I2C bus delay
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void i2c_delay(void)
{
    __IO uint32_t i;
    for(i = 0; i < 5 * 100; i++);
}

/*!
    \brief      generates an I2C start signal
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void i2c_start(void)
{
    /* I2C start signal is generated when SCL is high and SDA transitions from high to low */
    I2C_SDA_1();
    I2C_SCL_1();
    i2c_delay();
    I2C_SDA_0();
    i2c_delay();
    I2C_SCL_0();
    i2c_delay();
}

/*!
    \brief      generates an I2C stop signal
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void i2c_stop(void)
{
    /* I2C stop signal is generated when SCL is high and SDA transitions from low to high */
    I2C_SDA_0();
    I2C_SCL_1();
    i2c_delay();
    I2C_SDA_1();
}

/*!
    \brief      sends a byte via I2C bus
    \param[in]  byte: byte to be sent
    \param[out] none
    \retval     none
*/
static void i2c_sendbyte(uint8_t byte)
{
    __IO uint32_t i;

    /* send the byte, starting from the MSB (bit 7) */
    for(i = 0; i < 8; i++) {
        if(byte & 0x80) {
            I2C_SDA_1();
        } else {
            I2C_SDA_0();
        }
        i2c_delay();
        I2C_SCL_1();
        i2c_delay();
        I2C_SCL_0();
        if(i == 7) {
        /* release the bus */
            I2C_SDA_1();
        }
        /* shift left by one bit */
        byte <<= 1;
        i2c_delay();
    }
}

/*!
    \brief      reads a byte from the I2C bus
    \param[in]  none
    \param[out] none
    \retval     uint8_t: byte read from the bus
*/
static uint8_t i2c_readbyte(void)
{
    __IO uint32_t i;
    __IO uint8_t value;

    /* read the byte, starting from the MSB (bit 7) */
    value = 0;
    for(i = 0; i < 8; i++) {
        value <<= 1;
        I2C_SCL_1();
        i2c_delay();
        if(I2C_SDA_READ()) {
            value++;
        }
        I2C_SCL_0();
        i2c_delay();
    }
    return value;
}

/*!
    \brief      waits for ACK signal from the I2C device
    \param[in]  none
    \param[out] none
    \retval     uint8_t: returns 0 if ACK received, 1 otherwise
*/
static uint8_t i2c_waitack(void)
{
    __IO uint8_t re;
    /* release SDA bus */
    I2C_SDA_1();
    i2c_delay();
    /* drive SCL high, device will send ACK */
    I2C_SCL_1();
    i2c_delay();
    /* read SDA line status */
    if(I2C_SDA_READ()) {
        re = 1;
    } else {
        re = 0;
    }
    I2C_SCL_0();
    i2c_delay();
    return re;
}

/*!
    \brief      generates an ACK signal on the I2C bus
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void i2c_ack(void)
{
    /* drive SDA low */
    I2C_SDA_0();
    i2c_delay();
    /* generate a clock pulse */
    I2C_SCL_1();
    i2c_delay();
    I2C_SCL_0();
    i2c_delay();
    /* release SDA bus */
    I2C_SDA_1();
}

/*!
    \brief      generates a NACK signal on the I2C bus
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void i2c_nack(void)
{
    /* drive SDA high */
    I2C_SDA_1();
    i2c_delay();
    /* generate a clock pulse */
    I2C_SCL_1();
    i2c_delay();
    I2C_SCL_0();
    i2c_delay();
}

/*!
    \brief      configures the gpio pins for I2C communication
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void i2c_gpio_config(void)
{
    /* configure SCL pin */
    rcu_periph_clock_enable(GTP_I2C_SCL_GPIO_CLK);
    gpio_mode_set(GTP_I2C_SCL_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GTP_I2C_SCL_PIN);
    gpio_output_options_set(GTP_I2C_SCL_GPIO_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_60MHZ, GTP_I2C_SCL_PIN);

    /* configure SDA pin */
    rcu_periph_clock_enable(GTP_I2C_SDA_GPIO_CLK);
    gpio_mode_set(GTP_I2C_SDA_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GTP_I2C_SDA_PIN);
    gpio_output_options_set(GTP_I2C_SDA_GPIO_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_60MHZ, GTP_I2C_SDA_PIN);

    rcu_periph_clock_enable(GTP_RST_GPIO_CLK);
    rcu_periph_clock_enable(GTP_INT_GPIO_CLK);

    /* configure RST pin */
    gpio_mode_set(GTP_RST_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GTP_RST_GPIO_PIN);
    gpio_output_options_set(GTP_RST_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_12MHZ, GTP_RST_GPIO_PIN);

    /* configure INT pin */
    gpio_mode_set(GTP_INT_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GTP_INT_GPIO_PIN);
    gpio_output_options_set(GTP_INT_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_12MHZ, GTP_INT_GPIO_PIN);
}
