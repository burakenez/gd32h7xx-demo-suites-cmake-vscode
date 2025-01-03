/*!
    \file    spi_quad_lcd_driver.c
    \brief   spi quad lcd driver

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

#include "gd32h7xx.h"
#include "systick.h"
#include <stdio.h>
#include "gd32h759i_eval.h"
#include "spi_quad_lcd_driver.h"

/*!
    \brief      write command to the LCD register
    \param[in]  lcd_reg: selected register address
    \param[out] none
    \retval     none
*/
static void lcd_command_write(uint8_t lcd_reg)
{
    while(RESET == spi_i2s_flag_get(LCD_SPI, SPI_FLAG_TP));
    spi_i2s_data_transmit(LCD_SPI, lcd_reg);
    while(RESET == spi_i2s_flag_get(LCD_SPI, SPI_FLAG_TC));
}

/*!
    \brief      write data to the LCD register
    \param[in]  data: data to be written to the selected register
    \param[out] none
    \retval     none
*/
static void lcd_data_write(uint32_t data)
{
    while(RESET == spi_i2s_flag_get(LCD_SPI, SPI_FLAG_TP));
    spi_i2s_data_transmit(LCD_SPI, data);
    while(RESET == spi_i2s_flag_get(LCD_SPI, SPI_FLAG_TC));
}

/*!
    \brief      config_spi_quad_mode_enable
    \param[in]  spi_periph: SPIx(x=3,4)
    \param[out] none
    \retval     none
*/
static void config_spi_quad_mode_enable(uint32_t spi_periph)
{
    spi_disable(spi_periph);
    /* enable quad wire SPI */
    spi_quad_enable(spi_periph);
    /* enable quad wire SPI write */
    spi_quad_write_enable(spi_periph);
    /* disable byte mode */
    spi_byte_access_disable(spi_periph);
    /* enable word mode */
    spi_word_access_enable(spi_periph);
    /* FIFO level set 4-data frame */
    spi_fifo_threshold_level_set(spi_periph, SPI_FIFO_TH_04DATA);
    spi_enable(spi_periph);
}

/*!
    \brief      config_spi_quad_mode_disable
    \param[in]  spi_periph: SPIx(x=3,4)
    \param[out] none
    \retval     none
*/
static void config_spi_quad_mode_disable(uint32_t spi_periph)
{
    spi_disable(spi_periph);
    /* disable quad wire SPI */
    spi_quad_disable(spi_periph);
    /* disable word mode */
    spi_word_access_disable(spi_periph);
    /* enable byte mode */
    spi_byte_access_enable(spi_periph);
    /* FIFO level set 1-data frame */
    spi_fifo_threshold_level_set(spi_periph, SPI_FIFO_TH_01DATA);
    spi_enable(spi_periph);
}

/*!
    \brief      transmit command, write data display to be displayed
    \param[in]  cmd: command
    \param[in]  length: data length
    \param[in]  data_ptr: pointer to data to be transferred
    \param[out] none
    \retval     none
*/
static void spi_lcd_init_transmit(uint32_t cmd, uint32_t length, uint8_t *data_ptr)
{
    uint32_t i;

    SPI_LCD_CS_LOW();
    /* start SPI master transfer */
    spi_master_transfer_start(LCD_SPI, SPI_TRANS_START);
    if(length == 0) {
        lcd_command_write(0x02);
        lcd_command_write(0x00);
        lcd_command_write((uint8_t)cmd);
        lcd_command_write(0x00);
    } else {
        lcd_command_write(0x02);
        lcd_command_write(0x00);
        lcd_command_write((uint8_t)cmd);
        lcd_command_write(0x00);
        for(i = 0; i < length; i++) {
            lcd_command_write(data_ptr[i]);
        }
    }
    SPI_LCD_CS_HIGH();
}

/*!
    \brief      lcd initialize
    \param[in]  none
    \param[out] none
    \retval     none
*/
void spi_quad_lcd_init(void)
{
    uint32_t i;

    init_line_t *init = (init_line_t *)&init_table[0];

    for(i = 0; i < sizeof(init_table) / sizeof(init_line_t); i++) {
        /* transmit initialize code */
        if(init->cmd == 0xff) {
            /* delay */
            delay_ms(init->dat[0]);
        } else {
            /* transmit command */
            spi_lcd_init_transmit(init->cmd, init->len, init->dat);
        }
        init++;
    }
}

/*!
    \brief      set the lcd display range
    \param[in]  x_start: x coordinate start dot
    \param[in]  x_end: x coordinate end dot
    \param[in]  y_start: y coordinate start dot
    \param[in]  y_end: y coordinate end dot
    \param[out] none
    \retval     none
*/
static void block_write(uint16_t x_start, uint16_t x_end, uint16_t y_start, uint16_t y_end)
{
    SPI_LCD_CS_LOW();
    /* start SPI master transfer */
    spi_master_transfer_start(LCD_SPI, SPI_TRANS_START);
    lcd_command_write(0x02);
    lcd_command_write(0x00);
    lcd_command_write(0x2a);
    lcd_command_write(0x00);
    lcd_command_write(x_start >> 8);
    lcd_command_write(x_start & 0xff);
    lcd_command_write(x_end >> 8);
    lcd_command_write(x_end & 0xff);
    SPI_LCD_CS_HIGH();

    SPI_LCD_CS_LOW();
    lcd_command_write(0x02);
    lcd_command_write(0x00);
    lcd_command_write(0x2b);
    lcd_command_write(0x00);
    lcd_command_write(y_start >> 8);
    lcd_command_write(y_start & 0xff);
    lcd_command_write(y_end >> 8);
    lcd_command_write(y_end & 0xff);
    SPI_LCD_CS_HIGH();

    SPI_LCD_CS_LOW();
    lcd_command_write(0x02);
    lcd_command_write(0x00);
    lcd_command_write(0x2c);
    lcd_command_write(0x00);
    SPI_LCD_CS_HIGH();

    SPI_LCD_CS_LOW();
    lcd_command_write(0x32);
    lcd_command_write(0x00);
    lcd_command_write(0x2c);
    lcd_command_write(0x00);
    SPI_LCD_CS_HIGH();
}

/*!
    \brief      4-wire Pixel Write Data Waveform
    \param[in]  length: data length
    \param[in]  data_ptr: pointer to data to be transferred
    \param[out] none
    \retval     none
*/
static void write_lcd(uint32_t length, uint8_t *data_ptr)
{
    uint32_t i, m32word;
    config_spi_quad_mode_enable(LCD_SPI);
    /* start SPI master transfer */
    spi_master_transfer_start(LCD_SPI, SPI_TRANS_START);
    for(i = 0; i < length / 4; i++) {
        m32word = (uint32_t)((data_ptr[i * 4 + 3] << 24) | (data_ptr[i * 4 + 2] << 16) | (data_ptr[i * 4 + 1] << 8) | data_ptr[i * 4]);
        lcd_data_write(m32word);
    }
    config_spi_quad_mode_disable(LCD_SPI);
}

/*!
    \brief      lcd display picture
    \param[in]  picture: pointer to data to be transferred
    \param[out] none
    \retval     none
*/
void spi_quad_lcd_picture_disp(uint8_t *picture)
{
    block_write(0, LCD_PIXEL_WIDTH - 1, 0, LCD_PIXEL_HEIGHT - 1);
    SPI_LCD_CS_LOW();
    lcd_command_write(0x32);
    lcd_command_write(0x00);
    lcd_command_write(0x2c);
    lcd_command_write(0x00);
    write_lcd(LCD_PIXEL_HEIGHT * LCD_PIXEL_WIDTH * 2, picture);
    SPI_LCD_CS_HIGH();
}

/*!
    \brief      lcd display color
    \param[in]  color: color data to be transferred
    \param[out] none
    \retval     none
*/
void spi_quad_lcd_clean(uint16_t color)
{
    uint16_t i, j;
    uint8_t data0, data1;
    uint32_t m32word;
    block_write(0, LCD_PIXEL_WIDTH - 1, 0, LCD_PIXEL_HEIGHT - 1);
    data0 = (uint8_t)(color >> 8);
    data1 = (uint8_t)color;

    SPI_LCD_CS_LOW();
    lcd_command_write(0x32);
    lcd_command_write(0x00);
    lcd_command_write(0x2c);
    lcd_command_write(0x00);
    m32word = (uint32_t)((data1 << 24) | (data0 << 16) | (data1 << 8) | data0);
    config_spi_quad_mode_enable(LCD_SPI);
    /* start SPI master transfer */
    spi_master_transfer_start(LCD_SPI, SPI_TRANS_START);
    for(i = 0; i < LCD_PIXEL_HEIGHT; i++) {
        for(j = 0; j < LCD_PIXEL_WIDTH / 2; j++) {
            lcd_data_write(m32word);
        }
    }
    config_spi_quad_mode_disable(LCD_SPI);
    SPI_LCD_CS_HIGH();
}
