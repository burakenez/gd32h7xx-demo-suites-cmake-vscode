/*!
    \file    main.c
    \brief   SD card read and write demo

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
#include "sdcard.h"
#include <stdio.h>
#include "systick.h"
#include "gd32h759i_eval.h"

#define BUSMODE_1BIT    0
#define BUSMODE_4BIT    1

/* SDIO bus switch */
/* config SDIO bus mode, select: BUSMODE_1BIT/BUSMODE_4BIT */
#define SDIO_BUSMODE        BUSMODE_4BIT
/* config SDIO speed mode, select: SD_SPEED_DEFAULT/SD_SPEED_HIGH */
#define SDIO_SPEEDMODE      SD_SPEED_HIGH
/* config data transfer mode, select: SD_POLLING_MODE/SD_DMA_MODE */
#define SDIO_DTMODE         SD_POLLING_MODE

//#define DATA_PRINT                                          /* uncomment the macro to print out the data */
sd_card_info_struct sd_cardinfo;                            /* information of SD card */

uint32_t __attribute__((aligned(32))) buf_write[512];       /* store the data written to the card */
uint32_t __attribute__((aligned(32))) buf_read[512];        /* store the data read from the card */


void nvic_config(void);
sd_error_enum sd_io_init(void);
void card_info_get(void);
void cache_enable(void);
void led_flash(int times);
ErrStatus memory_compare(uint32_t *src, uint32_t *dst, uint16_t length);

int main()
{
    sd_error_enum sd_error;
    ErrStatus state = ERROR;

    uint16_t i = 5;
#ifdef DATA_PRINT
    uint8_t *pdata;
#endif /* DATA_PRINT */

    /* enable the CPU Cache */
    cache_enable();

    /* configure the NVIC and USART */
    nvic_config();
    systick_config();
    gd_eval_com_init(EVAL_COM);
    gd_eval_led_init(LED1);
    gd_eval_led_init(LED2);

    /* turn off all the LEDs */
    gd_eval_led_off(LED1);
    gd_eval_led_off(LED2);

    /* flash the LEDs for 1 time */
    led_flash(1);

    /* initialize the card */
    do {
        sd_error = sd_io_init();
    } while((SD_OK != sd_error) && (--i));

    if(i) {
        printf("\r\n Card init success!\r\n");
    } else {
        printf("\r\n Card init failed!\r\n");
        /* turn on LED1, LED2 */
        gd_eval_led_on(LED1);
        gd_eval_led_on(LED2);
        while(1) {
        }
    }

    /* get the information of the card and print it out by USART */
    card_info_get();


    /* init the write buffer */
    for(i = 0; i < 512; i++) {
        buf_write[i] = i;
    }

    /* clean and invalidate buffer in D-Cache */
    SCB_CleanInvalidateDCache_by_Addr(buf_write, 512 * 4);

    printf("\r\n\r\n Card test:");
    /* single block operation test */
    sd_error = sd_block_write(buf_write, 100, 512);
    if(SD_OK != sd_error) {
        printf("\r\n Block write fail!");
        /* turn on LED1, LED2 */
        gd_eval_led_on(LED1);
        gd_eval_led_on(LED2);
        while(1) {
        }
    } else {
        printf("\r\n Block write success!");
    }

    sd_error = sd_block_read(buf_read, 100, 512);
    if(SD_OK != sd_error) {
        printf("\r\n Block read fail!");
        /* turn on LED1, LED2 */
        gd_eval_led_on(LED1);
        gd_eval_led_on(LED2);
        while(1) {
        }
    } else {
        printf("\r\n Block read success!");
#ifdef DATA_PRINT
        SCB_CleanInvalidateDCache_by_Addr(buf_read, 512 * 4);
        pdata = (uint8_t *)buf_read;
        /* print data by USART */
        printf("\r\n");
        for(i = 0; i < 128; i++) {
            printf(" %3d %3d %3d %3d ", *pdata, *(pdata + 1), *(pdata + 2), *(pdata + 3));
            pdata += 4;
            if(0 == (i + 1) % 4) {
                printf("\r\n");
            }
        }
#endif /* DATA_PRINT */
    }

    /* compare the write date and the read data */
    state = memory_compare(buf_write, buf_read, 128);
    if(SUCCESS == state) {
        printf("\r\n Single block read compare successfully!");
    } else {
        printf("\r\n Single block read compare fail!");
        /* turn on LED1, LED2 */
        gd_eval_led_on(LED1);
        gd_eval_led_on(LED2);
        while(1) {
        }
    }

    /* lock and unlock operation test */
    if(SD_CCC_LOCK_CARD & sd_cardinfo.card_csd.ccc) {
        /* lock the card */
        sd_error = sd_lock_unlock(SD_LOCK);
        if(SD_OK != sd_error) {
            printf("\r\n Lock failed!");
            /* turn on LED1, LED2 */
            gd_eval_led_on(LED1);
            gd_eval_led_on(LED2);
            while(1) {
            }
        } else {
            printf("\r\n The card is locked!");
        }
        sd_error = sd_erase(100, 101);
        if(SD_OK != sd_error) {
            printf("\r\n Erase failed!");
        } else {
            printf("\r\n Erase success!");
        }

        /* unlock the card */
        sd_error = sd_lock_unlock(SD_UNLOCK);
        if(SD_OK != sd_error) {
            printf("\r\n Unlock failed!");
            /* turn on LED1, LED2 */
            gd_eval_led_on(LED1);
            gd_eval_led_on(LED2);
            while(1) {
            }
        } else {
            printf("\r\n The card is unlocked!");
        }
        sd_error = sd_erase(100, 101);
        if(SD_OK != sd_error) {
            printf("\r\n Erase failed!");
        } else {
            printf("\r\n Erase success!");
        }

        sd_error = sd_block_read(buf_read, 100, 512);
        if(SD_OK != sd_error) {
            printf("\r\n Block read fail!");
            /* turn on LED1, LED2 */
            gd_eval_led_on(LED1);
            gd_eval_led_on(LED2);
            while(1) {
            }
        } else {
            printf("\r\n Block read success!");
#ifdef DATA_PRINT
            SCB_CleanInvalidateDCache_by_Addr(buf_read, 512 * 4);
            pdata = (uint8_t *)buf_read;
            /* print data by USART */
            printf("\r\n");
            for(i = 0; i < 128; i++) {
                printf(" %3d %3d %3d %3d ", *pdata, *(pdata + 1), *(pdata + 2), *(pdata + 3));
                pdata += 4;
                if(0 == (i + 1) % 4) {
                    printf("\r\n");
                }
            }
#endif /* DATA_PRINT */
        }
    }

    /* multiple blocks operation test */
    sd_error = sd_multiblocks_write(buf_write, 200, 512, 3);
    if(SD_OK != sd_error) {
        printf("\r\n Multiple block write fail!");
        /* turn on LED1, LED2 */
        gd_eval_led_on(LED1);
        gd_eval_led_on(LED2);
        while(1) {
        }
    } else {
        printf("\r\n Multiple block write success!");
    }

    sd_error = sd_multiblocks_read(buf_read, 200, 512, 3);
    if(SD_OK != sd_error) {
        printf("\r\n Multiple block read fail!");
        /* turn on LED1, LED2 */
        gd_eval_led_on(LED1);
        gd_eval_led_on(LED2);
        while(1) {
        }
    } else {
        printf("\r\n Multiple block read success!");
#ifdef DATA_PRINT
        SCB_CleanInvalidateDCache_by_Addr(buf_read, 512 * 4);
        pdata = (uint8_t *)buf_read;
        /* print data by USART */
        printf("\r\n");
        for(i = 0; i < 512; i++) {
            printf(" %3d %3d %3d %3d ", *pdata, *(pdata + 1), *(pdata + 2), *(pdata + 3));
            pdata += 4;
            if(0 == (i + 1) % 4) {
                printf("\r\n");
            }
        }
#endif /* DATA_PRINT */
    }
    /* compare the write date and the read data */
    state = memory_compare(buf_write, buf_read, 128 * 3);
    if(SUCCESS == state) {
        printf("\r\n Multiple block read compare successfully!");
    } else {
        printf("\r\n Multiple block read compare fail!");
        /* turn on LED1, LED2 */
        gd_eval_led_on(LED1);
        gd_eval_led_on(LED2);
        while(1) {
        }
    }
    printf("\r\n SD card test successfully!");

    while(1) {};
}

/*!
    \brief      configure the NVIC
    \param[in]  none
    \param[out] none
    \retval     none
*/
void nvic_config(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(SDIO0_IRQn, 2U, 0);
}

/*!
  \brief      flash the LEDs for test
  \param[in]  times: times to flash the LEDs
  \param[out] none
  \retval     none
*/
void led_flash(int times)
{
    int i;
    for(i = 0; i < times; i++) {
        /* delay 400 ms */
        delay_ms(400);

        /* turn on LEDs */
        gd_eval_led_on(LED1);
        gd_eval_led_on(LED2);

        /* delay 400 ms */
        delay_ms(400);

        /* turn off LEDs */
        gd_eval_led_off(LED1);
        gd_eval_led_off(LED2);
    }
}

/*!
    \brief      initialize the card, get the card information, set the bus mode and transfer mode
    \param[in]  none
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_io_init(void)
{
    sd_error_enum status = SD_OK;
    uint32_t cardstate = 0;

    status = sd_init();
    if(SD_OK == status) {
        status = sd_card_information_get(&sd_cardinfo);
    }
    if(SD_OK == status) {
        status = sd_card_select_deselect(sd_cardinfo.card_rca);
    }
    status = sd_cardstatus_get(&cardstate);
    if(cardstate & 0x02000000) {
        printf("\r\n the card is locked!");
        status = sd_lock_unlock(SD_UNLOCK);
        if(status != SD_OK) {
            return SD_LOCK_UNLOCK_FAILED;
        } else {
            printf("\r\n the card is unlocked successfully!");
        }
    }

    if((SD_OK == status) && (!(cardstate & 0x02000000))) {
        /* set bus mode */
#if (SDIO_BUSMODE == BUSMODE_4BIT)
        status = sd_bus_mode_config(SDIO_BUSMODE_4BIT, SDIO_SPEEDMODE);
#else
        status = sd_bus_mode_config(SDIO_BUSMODE_1BIT, SDIO_SPEEDMODE);
#endif
    }

    if(SD_OK == status) {
        /* set data transfer mode */
        status = sd_transfer_mode_config(SDIO_DTMODE);
    }
    return status;
}

/*!
    \brief      get the card information and print it out by USRAT
    \param[in]  none
    \param[out] none
    \retval     none
*/
void card_info_get(void)
{
    uint8_t sd_spec, sd_spec3, sd_spec4, sd_security;
    uint32_t block_count, block_size, sd_capacity;
    uint16_t temp_ccc;
    printf("\r\n Card information:");
    sd_spec = (sd_scr[1] & 0x0F000000) >> 24;
    sd_spec3 = (sd_scr[1] & 0x00008000) >> 15;
    sd_spec4 = (sd_scr[1] & 0x00000400) >> 10;
    if(2 == sd_spec) {
        if(1 == sd_spec3) {
            if(1 == sd_spec4) {
                printf("\r\n## Card version 4.xx ##");
            } else {
                printf("\r\n## Card version 3.0x ##");
            }
        } else {
            printf("\r\n## Card version 2.00 ##");
        }
    } else if(1 == sd_spec) {
        printf("\r\n## Card version 1.10 ##");
    } else if(0 == sd_spec) {
        printf("\r\n## Card version 1.0x ##");
    }

    sd_security = (sd_scr[1] & 0x00700000) >> 20;
    if(2 == sd_security) {
        printf("\r\n## security v1.01 ##");
    } else if(3 == sd_security) {
        printf("\r\n## security v2.00 ##");
    } else if(4 == sd_security) {
        printf("\r\n## security v3.00 ##");
    }

    block_size = 512U;
    sd_capacity = sd_card_capacity_get();
    block_count = sd_capacity * 2U;
    printf("\r\n## Device size is %dKB ##", sd_capacity);
    printf("\r\n## Block size is %dB ##", block_size);
    printf("\r\n## Block count is %d ##", block_count);

    if(sd_cardinfo.card_csd.read_bl_partial) {
        printf("\r\n## Partial blocks for read allowed ##");
    }
    if(sd_cardinfo.card_csd.write_bl_partial) {
        printf("\r\n## Partial blocks for write allowed ##");
    }
    temp_ccc = sd_cardinfo.card_csd.ccc;
    printf("\r\n## CardCommandClasses is: %x ##", temp_ccc);
    if((SD_CCC_BLOCK_READ & temp_ccc) && (SD_CCC_BLOCK_WRITE & temp_ccc)) {
        printf("\r\n## Block operation supported ##");
    }
    if(SD_CCC_ERASE & temp_ccc) {
        printf("\r\n## Erase supported ##");
    }
    if(SD_CCC_WRITE_PROTECTION & temp_ccc) {
        printf("\r\n## Write protection supported ##");
    }
    if(SD_CCC_LOCK_CARD & temp_ccc) {
        printf("\r\n## Lock unlock supported ##");
    }
    if(SD_CCC_APPLICATION_SPECIFIC & temp_ccc) {
        printf("\r\n## Application specific supported ##");
    }
    if(SD_CCC_IO_MODE & temp_ccc) {
        printf("\r\n## I/O mode supported ##");
    }
    if(SD_CCC_SWITCH & temp_ccc) {
        printf("\r\n## Switch function supported ##");
    }
}

/*!
    \brief      memory compare function
    \param[in]  src : source data
    \param[in]  dst : destination data
    \param[in]  length : the compare data length
    \param[out] none
    \retval     ErrStatus : ERROR or SUCCESS
*/
ErrStatus memory_compare(uint32_t *src, uint32_t *dst, uint16_t length)
{
    while(length--) {
        if(*src++ != *dst++) {
            return ERROR;
        }
    }
    return SUCCESS;
}

#ifdef __GNUC__
/* retarget the C library printf function to the USART, in Eclipse GCC environment */
int __io_putchar(int ch)
{
    usart_data_transmit(EVAL_COM, (uint8_t) ch );
    while(RESET == usart_flag_get(EVAL_COM, USART_FLAG_TBE));
    return ch;
}
#else
/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM, (uint8_t)ch);
    while(RESET == usart_flag_get(EVAL_COM, USART_FLAG_TBE));

    return ch;
}
#endif /* __GNUC__ */

/*!
    \brief      enable the CPU cache
    \param[in]  none
    \param[out] none
    \retval     none
*/
void cache_enable(void)
{
    /* enable I-Cache */
    SCB_EnableICache();

    /* enable D-Cache */
    SCB_EnableDCache();
}
