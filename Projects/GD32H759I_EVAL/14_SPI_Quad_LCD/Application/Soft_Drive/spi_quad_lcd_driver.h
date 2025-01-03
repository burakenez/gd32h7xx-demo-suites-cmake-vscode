/*!
    \file    spi_quad_lcd_driver.h
    \brief   the header file of QSPI_LCD_driver

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

#ifndef SPI_QUAD_LCD_DRIVER_H
#define SPI_QUAD_LCD_DRIVER_H

#include "gd32h7xx.h"
#include "GC9B71-H0189S001-320X386.h"

#define LCD_SPI                SPI4

#define LCD_RST_PIN            GPIO_PIN_12
#define LCD_RST_PORT           GPIOG
#define LCD_LEDK_PIN           GPIO_PIN_13
#define LCD_LEDK_PORT          GPIOG
#define LCD_CS_PIN             GPIO_PIN_6
#define LCD_CS_PORT            GPIOF

#define LCD_PIXEL_WIDTH        (320)
#define LCD_PIXEL_HEIGHT       (386)

#define LCD_COLOR_RED          0xF800
#define LCD_COLOR_GREEN        0x07E0
#define LCD_COLOR_BLUE         0x001F
#define LCD_COLOR_WHITE        0xFFFF
#define LCD_COLOR_BLACK        0x0000

#define SPI_LCD_CS_LOW()       gpio_bit_reset(LCD_CS_PORT, LCD_CS_PIN)
#define SPI_LCD_CS_HIGH()      gpio_bit_set(LCD_CS_PORT, LCD_CS_PIN)

/* lcd initialize */
void spi_quad_lcd_init(void);
/* clear the LCD with specified color */
void spi_quad_lcd_clean(uint16_t color);
/* lcd display picture */
void spi_quad_lcd_picture_disp(uint8_t *picture);

#endif /* SPI_QUAD_LCD_DRIVER_H */
