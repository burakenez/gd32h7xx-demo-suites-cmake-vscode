/*!
    \file    flash_msd.c
    \brief   flash access functions

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

#include "flash_msd.h"

/* pages 0 and 1 base and end addresses */
#define FLASH_BASE_ADDR         0x8010000U
#define PAGE_SIZE               0x1000U

/*!
    \brief      initialize the internal flash
    \param[in]  none
    \param[out] none
    \retval     status
  */
uint32_t flash_init()
{
    fmc_unlock();

    return 0U;
}

/*!
    \brief      read data from multiple blocks of internal flash
    \param[in]  pBuf: pointer to user buffer
    \param[in]  read_addr: address to be read
    \param[in]  block_size: size of block
    \param[in]  block_num: number of block
    \param[out] none
    \retval     status
*/
uint32_t flash_multi_blocks_read(uint8_t *buf, uint32_t read_addr, uint16_t block_size, uint32_t block_num)
{
    uint32_t i;
    uint8_t *src = (uint8_t *)(read_addr + FLASH_BASE_ADDR);

    /* Data transfer */
    while(block_num--) {
        for(i = 0U; i < block_size; i++) {
            *buf++ = *src++;
        }
    }

    return 0U;
}

/*!
    \brief      write data to multiple blocks of flash
    \param[in]  pBuf: pointer to user buffer
    \param[in]  write_addr: address to be write
    \param[in]  block_size: block size
    \param[in]  block_num: number of block
    \param[out] none
    \retval     status
*/
uint32_t flash_multi_blocks_write(uint8_t *buf, uint32_t write_addr, uint16_t block_size, uint32_t block_num)
{
    uint32_t i, page;
    uint32_t start_page = (write_addr / PAGE_SIZE) * PAGE_SIZE + FLASH_BASE_ADDR;
    uint32_t *ptrs = (uint32_t *)buf;

    page = block_num;

    for(; page > 0U; page--) {
        fmc_sector_erase(start_page);

        i = 0U;

        do {
            fmc_word_program(start_page, *ptrs++);
            start_page += 4U;
        } while(++i < 1024U);
    }

    return 0U;
}
