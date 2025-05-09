/*!
    \file    main.c
    \brief   running LED

    \version 2025-02-19, V2.1.0, OSPI 1&8 lines indirect/memory mapped read and write
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
#include "gd32h759i_eval.h"
#include <stdio.h>
#include "gd25x512me.h"
#include "systick.h"

#define countof(a)                 (sizeof(a) / sizeof(*(a)))
#define buffersize1                (countof(tx_buffer1))
#define buffersize2                (countof(tx_buffer2))
#define buffersize3                (countof(tx_buffer3))

#define GD25X512ME_ID              0xC8481AFF
#define OSPI_INTERFACE             OSPI0
//#define OSPI_INTERFACE             OSPI1

#define FLASH_WRITE_ADDRESS_1      0x000000
#define FLASH_READ_ADDRESS_1       FLASH_WRITE_ADDRESS1
#define FLASH_WRITE_ADDRESS_2      0x200000
#define FLASH_READ_ADDRESS_2       FLASH_WRITE_ADDRESS2

#define FLASH_WRITE_ADDRESS_3      (uint32_t)0x90400000
#define FLASH_READ_ADDRESS_3       FLASH_WRITE_ADDRESS3
//#define FLASH_WRITE_ADDRESS_3      (uint32_t)0x70400000
//#define FLASH_READ_ADDRESS_3       FLASH_WRITE_ADDRESS3

uint8_t tx_buffer1[] = "GD32H759I_EVAL octal-flash SPI mode with 1 line in indirect mode read&write test!\r\n";
uint8_t tx_buffer2[] = "GD32H759I_EVAL octal-flash OSPI mode with 8 lines in indirect mode read&write test!\r\n";
uint8_t tx_buffer3[] = "GD32H759I_EVAL octal-flash memory mapped read test!\r\n";
uint8_t rx_buffer1[buffersize1];
uint8_t rx_buffer2[buffersize2];
uint8_t rx_buffer3[buffersize3];

ospi_parameter_struct ospi_struct = {0};
uint32_t flashid = 0;
uint8_t i = 0;

void cache_enable(void);
ErrStatus memory_compare(uint8_t *src, uint8_t *dst, uint16_t length);
void memory_mapped_write(uint8_t *pdata, uint32_t address, uint32_t size);
void memory_mapped_read(uint8_t *pdata, uint32_t address, uint32_t size);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    /* enable the CPU Cache */
    cache_enable();
    
    /* configure systick */
    systick_config();
    
   /* configure USART */
    gd_eval_com_init(EVAL_COM);
    
    gd_eval_led_init(LED1);
    gd_eval_led_init(LED2);
    
    printf("\n\r#####################################################################################");
    printf("\n\rOSPI read flash ID and read/write with 1&8 lines in indirect/memory mapped mode test!\n\r");
    
    /* initialize OSPI/OSPIM  and GPIO */
    ospi_flash_init(OSPI_INTERFACE, &ospi_struct);
    
    /* reset ospi flash */
    ospi_flash_reset_enable(OSPI_INTERFACE, &ospi_struct, SPI_MODE);
    ospi_flash_reset_memory(OSPI_INTERFACE, &ospi_struct, SPI_MODE);
    ospi_flash_reset_enable(OSPI_INTERFACE, &ospi_struct, OSPI_MODE);
    ospi_flash_reset_memory(OSPI_INTERFACE, &ospi_struct, OSPI_MODE);
    
    /* read flash ID */
    flashid = ospi_flash_read_id(OSPI_INTERFACE, &ospi_struct, SPI_MODE);
    
    if(GD25X512ME_ID == flashid) {
        printf("\n\rThe device ID is 0x%X\n\r", flashid);
        
        /* 1 line in indirect mode read/write */
        printf("\n\rThe data written with 1 line in indirect mode to flash is:");
        printf("\n%s\n\r", tx_buffer1);
        
        /* erase specified address */
        ospi_flash_autopolling_mem_ready(OSPI_INTERFACE, &ospi_struct, SPI_MODE);
        ospi_flash_write_enbale(OSPI_INTERFACE, &ospi_struct, SPI_MODE);
        ospi_flash_block_erase(OSPI_INTERFACE, &ospi_struct, SPI_MODE, GD25X512ME_3BYTES_SIZE, FLASH_WRITE_ADDRESS_1, GD25X512ME_ERASE_4K);
        ospi_flash_autopolling_mem_ready(OSPI_INTERFACE, &ospi_struct, SPI_MODE);
        
        /* write data of tx_buffer1 to flash */
        ospi_flash_autopolling_mem_ready(OSPI_INTERFACE, &ospi_struct, SPI_MODE);
        ospi_flash_write_enbale(OSPI_INTERFACE, &ospi_struct, SPI_MODE);
        ospi_flash_page_program(OSPI_INTERFACE, &ospi_struct, SPI_MODE, GD25X512ME_3BYTES_SIZE, tx_buffer1, FLASH_WRITE_ADDRESS_1, buffersize1);
        ospi_flash_autopolling_mem_ready(OSPI_INTERFACE, &ospi_struct, SPI_MODE);
        
        /* read data from flash */
        ospi_flash_read(OSPI_INTERFACE, &ospi_struct, SPI_MODE, GD25X512ME_3BYTES_SIZE, rx_buffer1, FLASH_WRITE_ADDRESS_1, buffersize1);
        
        if(ERROR != memory_compare(tx_buffer1, rx_buffer1, buffersize1)) {
            printf("The data read with 1 line in indirect mode from flash is:");
            printf("\n%s\n", tx_buffer1);
            printf("OSPI read/write with 1 line in indirect test success!\r\n");
        } else {
            printf("OSPI read/write with 1 line in indirect test failed!\r\n");
            while(1){
            }
        }
        
        /* 8 lines in indirect mode read/write */
        printf("\n\rThe data written with 8 lines in indirect mode to flash is:\n");

        printf("\n%s\n\r", tx_buffer2);
        
        /* configure OSPI FLASH dummy cycles */
        ospi_flash_write_enbale(OSPI_INTERFACE, &ospi_struct, SPI_MODE);
        ospi_flash_write_volatilecfg_register(OSPI_INTERFACE, &ospi_struct, SPI_MODE, GD25X512ME_3BYTES_SIZE, GD25X512ME_CFG_REG1_ADDR, GD25X512ME_CFG_16_DUMMY_CYCLES);
        
        /* configure OSPI FLASH enter STR OSPI mode */
        ospi_flash_write_enbale(OSPI_INTERFACE, &ospi_struct, SPI_MODE);
        ospi_flash_write_volatilecfg_register(OSPI_INTERFACE, &ospi_struct, SPI_MODE, GD25X512ME_3BYTES_SIZE, GD25X512ME_CFG_REG0_ADDR, GD25X512ME_CFG_OCTAL_STR_WO);
        ospi_flash_autopolling_mem_ready(OSPI_INTERFACE, &ospi_struct, OSPI_MODE);
        
        /* erase specified address */
        ospi_flash_autopolling_mem_ready(OSPI_INTERFACE, &ospi_struct, OSPI_MODE);
        ospi_flash_write_enbale(OSPI_INTERFACE, &ospi_struct, OSPI_MODE);
        ospi_flash_block_erase(OSPI_INTERFACE, &ospi_struct, OSPI_MODE, GD25X512ME_3BYTES_SIZE, FLASH_WRITE_ADDRESS_2, GD25X512ME_ERASE_4K);
        ospi_flash_autopolling_mem_ready(OSPI_INTERFACE, &ospi_struct, OSPI_MODE);
        
        /* write data of tx_buffer to flash */
        ospi_flash_autopolling_mem_ready(OSPI_INTERFACE, &ospi_struct, OSPI_MODE);
        ospi_flash_write_enbale(OSPI_INTERFACE, &ospi_struct, OSPI_MODE);
        ospi_flash_page_program(OSPI_INTERFACE, &ospi_struct, OSPI_MODE, GD25X512ME_3BYTES_SIZE, tx_buffer2, FLASH_WRITE_ADDRESS_2, buffersize2);
        ospi_flash_autopolling_mem_ready(OSPI_INTERFACE, &ospi_struct, OSPI_MODE);
        
        /* read data from flash */
        ospi_flash_read(OSPI_INTERFACE, &ospi_struct, OSPI_MODE, GD25X512ME_3BYTES_SIZE, rx_buffer2, FLASH_WRITE_ADDRESS_2, buffersize2);
        
        if(ERROR != memory_compare(tx_buffer2, rx_buffer2, buffersize2)) {
             printf("\n\rThe data read with 8 lines in indirect mode from flash is:\n");
             for(i = 0; i < buffersize2; i++) {
                printf("%c", rx_buffer2[i]);
             }
             printf("\r\n");
             printf("OSPI read/write with 8 lines in indirect test success!\r\n");
        } else {
            printf("OSPI read/write with 8 lines in indirect test failed!\r\n");
            while(1){
            }
        }
        
        /* memory mapped mode read/write */
        printf("\n\rThe data written in indirect mode to flash is:\n");
        for(i = 0; i < buffersize3; i++){
            printf("%c", tx_buffer3[i]);
        }
        /* erase specified address */
        ospi_flash_autopolling_mem_ready(OSPI_INTERFACE, &ospi_struct, OSPI_MODE);
        ospi_flash_write_enbale(OSPI_INTERFACE, &ospi_struct, OSPI_MODE);
        ospi_flash_block_erase(OSPI_INTERFACE, &ospi_struct, OSPI_MODE, GD25X512ME_3BYTES_SIZE, 0x400000, GD25X512ME_ERASE_4K);
        
        /* write data of tx_buffer to flash */
        ospi_flash_autopolling_mem_ready(OSPI_INTERFACE, &ospi_struct, OSPI_MODE);
        ospi_flash_write_enbale(OSPI_INTERFACE, &ospi_struct, OSPI_MODE);
        ospi_flash_memory_map_mode_wrap_enable(OSPI_INTERFACE, &ospi_struct, OSPI_MODE, GD25X512ME_3BYTES_SIZE);
        ospi_flash_page_program(OSPI_INTERFACE, &ospi_struct, OSPI_MODE, GD25X512ME_3BYTES_SIZE, tx_buffer3, 0x400000, buffersize3);
        ospi_flash_autopolling_mem_ready(OSPI_INTERFACE, &ospi_struct, OSPI_MODE);

        ospi_flash_memory_map_mode_wrap_enable(OSPI_INTERFACE, &ospi_struct, OSPI_MODE, GD25X512ME_3BYTES_SIZE);
        memory_mapped_read(rx_buffer3, FLASH_WRITE_ADDRESS_3, buffersize3);
        
        if(ERROR != memory_compare(tx_buffer3, rx_buffer3, buffersize3)) {
            printf("\n\rThe data read in memory mapped mode from flash is:\n");
            for(i = 0; i < buffersize3; i++) {
               printf("%c", rx_buffer3[i]);
            }
            printf("\r\n");
            printf("OSPI read in memory mapped mode test success!\r\n");
        } else {
            printf("OSPI read in memory mapped mode test failed!\r\n");
            while(1){
            }
        }
        
    } else {
        printf("\n\rFailed to read device ID\n\r");
        while(1){
        }
    }
    
    while(1){
        /* turn on LED2 */
        gd_eval_led_on(LED1);
        delay_ms(300);
        
        /* turn on LED3 */
        gd_eval_led_on(LED2);
        delay_ms(300);
        
        /* turn off LED2 */
        gd_eval_led_off(LED1);
        delay_ms(300);
        
        /* turn off LED3 */
        gd_eval_led_off(LED2);
        delay_ms(300);
    }
}

/*!
    \brief      enable the CPU Chache
    \param[in]  none
    \param[out] none
    \retval     none
*/
void cache_enable(void)
{
    /* Enable I-Cache */
    SCB_EnableICache();

    /* Enable D-Cache */
    SCB_EnableDCache();
}

/*!
    \brief      read in memory mapped mode
    \param[in]  pdata: pointer to data to be read
    \param[in]  address: read start address
    \param[in]  size: the size of read
    \param[out] none
    \retval     none
*/
void memory_mapped_read(uint8_t *pdata, uint32_t address, uint32_t size)
{
    for(i=0; i<size; i++){
        pdata[i] = *(uint8_t *)address++;
    }
}

/*!
    \brief      memory compare function
    \param[in]  src: source data pointer
    \param[in]  dst: destination data pointer
    \param[in]  length: the compare data length
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus memory_compare(uint8_t *src, uint8_t *dst, uint16_t length)
{
    while(length --) {
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
