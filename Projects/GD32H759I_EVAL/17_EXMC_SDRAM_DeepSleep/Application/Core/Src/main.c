/*!
    \file    main.c
    \brief   EXMC SDRAM Deepsleep demo

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
#include "exmc_sdram.h"

/* SDRAM */
#define BUFFER_SIZE                ((uint32_t)0x8000)
#define WRITE_READ_ADDR            ((uint32_t)0x7FFF00)

#define _PLL0PSC                   5U
#define _PLL0N                     (120U - 1U)
#define _PLL0P                     (1U - 1U)
#define _PLL0Q                     (2U - 1U)
#define _PLL0R                     (2U - 1U)

#define _PLL0PSC_REG_OFFSET        0U
#define _PLL0N_REG_OFFSET          6U
#define _PLL0P_REG_OFFSET          16U
#define _PLL0Q_REG_OFFSET          0U
#define _PLL0R_REG_OFFSET          24U

uint32_t writereadstatus = 0;
__ALIGNED(32) uint8_t txbuffer[BUFFER_SIZE];
__ALIGNED(32) uint8_t rxbuffer[BUFFER_SIZE];

static void cache_enable(void);
static void mpu_config(void);

void system_clock_configuration(void);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    exmc_sdram_command_parameter_struct     sdram_command_init_struct;
    uint16_t i = 0;

    /* enable the CPU cache */
    cache_enable();
    /* configure MPU */
    mpu_config();

    /* initialize LEDs */
    gd_eval_led_init(LED1);
    gd_eval_led_init(LED2);

    /* enable PMU clock */
    rcu_periph_clock_enable(RCU_PMU);

    /* initialize key */
    gd_eval_key_init(KEY_WAKEUP, KEY_MODE_EXTI);

    /* configure systick clock */
    systick_config();

    /* configure the USART */
    gd_eval_com_init(EVAL_COM);

    /* configure the EXMC access mode */
    exmc_synchronous_dynamic_ram_init(EXMC_SDRAM_DEVICE0);

    printf("\r\n\r\nSDRAM initialized!");
    delay_ms(1000);

    /* fill txbuffer */
    fill_buffer(txbuffer, BUFFER_SIZE, 0x0000);

    printf("\r\nSDRAM write data completed!");
    delay_ms(1000);

    /* write data to SDRAM */
    sdram_writebuffer_8(EXMC_SDRAM_DEVICE0, txbuffer, WRITE_READ_ADDR, BUFFER_SIZE);

    /* enter self-refresh mode */
    sdram_command_init_struct.command = EXMC_SDRAM_SELF_REFRESH;
    sdram_command_init_struct.bank_select = EXMC_SDRAM_DEVICE0_SELECT;
    sdram_command_init_struct.auto_refresh_number = EXMC_SDRAM_AUTO_REFLESH_1_SDCLK;
    sdram_command_init_struct.mode_register_content = 0;

    while(exmc_flag_get(EXMC_SDRAM_DEVICE0, EXMC_SDRAM_FLAG_NREADY) == SET);
    /* send the command */
    exmc_sdram_command_config(&sdram_command_init_struct);
    /* wait until the SDRAM controller is ready */
    while(exmc_flag_get(EXMC_SDRAM_DEVICE0, EXMC_SDRAM_FLAG_NREADY) == SET);
    /* check SDRAM self-refresh status */
    while(exmc_sdram_bankstatus_get(EXMC_SDRAM_DEVICE0) != EXMC_SDRAM_DEVICE_SELF_REFRESH);

    printf("\r\nEnter deepsleep mode!");
    delay_ms(1000);
    printf("\r\nPress the Wakeup key to wakeup the MCU!\r\n");
    delay_ms(1000);

    /* enter deepsleep mode, light on LED1 */
    gd_eval_led_on(LED1);
    pmu_to_deepsleepmode(WFI_CMD);
    gd_eval_led_off(LED1);

    /* reinitialize the system clock */
    system_clock_configuration();

    printf("\r\nWakeup key has been pressed!");
    delay_ms(1000);

    /* return normal mode */
    sdram_command_init_struct.command = EXMC_SDRAM_NORMAL_OPERATION;
    sdram_command_init_struct.bank_select = EXMC_SDRAM_DEVICE0;
    sdram_command_init_struct.auto_refresh_number = EXMC_SDRAM_AUTO_REFLESH_2_SDCLK;
    sdram_command_init_struct.mode_register_content = 0;
    /* send the command */
    exmc_sdram_command_config(&sdram_command_init_struct);
    /* wait until the SDRAM controller is ready */
    while(exmc_flag_get(EXMC_SDRAM_DEVICE0, EXMC_SDRAM_FLAG_NREADY) != RESET)
        /* check SDRAM self-refresh status */
        while(exmc_sdram_bankstatus_get(EXMC_SDRAM_DEVICE0) != EXMC_SDRAM_DEVICE_NORMAL);

    printf("\r\nSDRAM read data completed!");
    delay_ms(1000);

    /* read data from SDRAM */
    sdram_readbuffer_8(EXMC_SDRAM_DEVICE0, rxbuffer, WRITE_READ_ADDR, BUFFER_SIZE);

    printf("\r\nCheck the data!");
    delay_ms(1000);

    /* compare two buffers */
    for(i = 0; i < BUFFER_SIZE; i++) {
        if(rxbuffer[i] != txbuffer[i]) {
            writereadstatus ++;
            break;
        }
    }

    if(writereadstatus) {
        printf("\r\nSDRAM test failed!");

        /* failure, light off LED2 */
        gd_eval_led_off(LED2);
    } else {
        printf("\r\nSDRAM test succeeded!");
        delay_ms(1000);
        printf("\r\nThe data is:\r\n");
        delay_ms(1000);
        for(i = 0; i < BUFFER_SIZE; i++) {
            printf("%6x", rxbuffer[i]);
            if(((i + 1) % 16) == 0) {
                printf("\r\n");
            }
        }
        /* success, light on LED2 */
        gd_eval_led_on(LED2);
    }

    while(1);
}

/*!
    \brief      reconfigure the system clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void system_clock_configuration(void)
{
    uint32_t timeout = 0U;
    uint32_t stab_flag = 0U;

    /* enable HXTAL */
    RCU_CTL |= RCU_CTL_HXTALEN;

    /* wait until HXTAL is stable or the startup time is longer than HXTAL_STARTUP_TIMEOUT */
    do {
        timeout++;
        stab_flag = (RCU_CTL & RCU_CTL_HXTALSTB);
    } while((0U == stab_flag) && (HXTAL_STARTUP_TIMEOUT != timeout));
    /* if fail */
    if(0U == (RCU_CTL & RCU_CTL_HXTALSTB)) {
        while(1) {
        }
    }

    /* HXTAL is stable */
    /* AHB = SYSCLK / 2 */
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV2;
    /* APB4 = AHB / 2 */
    RCU_CFG0 |= RCU_APB4_CKAHB_DIV2;
    /* APB3 = AHB / 2 */
    RCU_CFG0 |= RCU_APB3_CKAHB_DIV2;
    /* APB2 = AHB / 1 */
    RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
    /* APB1 = AHB / 2 */
    RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;

    /* PLL0 select HXTAL, configure PLL0 input and output range */
    RCU_PLLALL &= ~(RCU_PLLALL_PLLSEL | RCU_PLLALL_PLL0VCOSEL | RCU_PLLALL_PLL0RNG);
    RCU_PLLALL |= (RCU_PLLSRC_HXTAL | RCU_PLL0RNG_4M_8M);

    /* PLL0P = HXTAL / 5 * 120 / 1 = 600 MHz */
    RCU_PLL0 &= ~(RCU_PLL0_PLL0N | RCU_PLL0_PLL0PSC | RCU_PLL0_PLL0P | RCU_PLL0_PLL0R | RCU_PLL0_PLLSTBSRC);
    RCU_PLL0 |= ((_PLL0N << _PLL0N_REG_OFFSET) | (_PLL0PSC << _PLL0PSC_REG_OFFSET) | (_PLL0P << _PLL0P_REG_OFFSET) | (_PLL0R << _PLL0R_REG_OFFSET));
    RCU_PLLADDCTL &= ~(RCU_PLLADDCTL_PLL0Q);
    RCU_PLLADDCTL |= (_PLL0Q << _PLL0Q_REG_OFFSET);

    /* enable PLL0P, PLL0Q, PLL0R */
    RCU_PLLADDCTL |= RCU_PLLADDCTL_PLL0PEN | RCU_PLLADDCTL_PLL0QEN | RCU_PLLADDCTL_PLL0REN;

    /* enable PLL0 */
    RCU_CTL |= RCU_CTL_PLL0EN;

    /* wait until PLL is stable */
    while(0U == (RCU_CTL & RCU_CTL_PLL0STB)) {
    }

    /* select PLL0 as system clock */
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_PLL0P;

    /* wait until PLL0 is selected as system clock */
    while(RCU_SCSS_PLL0P != (RCU_CFG0 & RCU_CFG0_SCSS)) {
    }
}

/*!
    \brief      enable the CPU cache
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void cache_enable(void)
{
    /* enable I-Cache and D-Cache */
    SCB_EnableICache();
    SCB_EnableDCache();
}

/*!
    \brief      configure the MPU attributes
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void mpu_config(void)
{
    mpu_region_init_struct mpu_init_struct;
    mpu_region_struct_para_init(&mpu_init_struct);

    /* disable the MPU */
    ARM_MPU_Disable();
    ARM_MPU_SetRegion(0, 0);
    
    /* configure the MPU attributes for Reserved, no access */
    mpu_init_struct.region_base_address  = 0x0;
    mpu_init_struct.region_size          = MPU_REGION_SIZE_4GB;
    mpu_init_struct.access_permission    = MPU_AP_NO_ACCESS;
    mpu_init_struct.access_bufferable    = MPU_ACCESS_NON_BUFFERABLE;
    mpu_init_struct.access_cacheable     = MPU_ACCESS_NON_CACHEABLE;
    mpu_init_struct.access_shareable     = MPU_ACCESS_SHAREABLE;
    mpu_init_struct.region_number        = MPU_REGION_NUMBER0;
    mpu_init_struct.subregion_disable    = 0x87;
    mpu_init_struct.instruction_exec     = MPU_INSTRUCTION_EXEC_NOT_PERMIT;
    mpu_init_struct.tex_type             = MPU_TEX_TYPE0;
    mpu_region_config(&mpu_init_struct);
    mpu_region_enable();
    
    /* configure the MPU attributes for SDRAM */
    mpu_init_struct.region_base_address  = 0xC0000000;
    mpu_init_struct.region_size          = MPU_REGION_SIZE_32MB;
    mpu_init_struct.access_permission    = MPU_AP_FULL_ACCESS;
    mpu_init_struct.access_bufferable    = MPU_ACCESS_NON_BUFFERABLE;
    mpu_init_struct.access_cacheable     = MPU_ACCESS_CACHEABLE;
    mpu_init_struct.access_shareable     = MPU_ACCESS_NON_SHAREABLE;
    mpu_init_struct.region_number        = MPU_REGION_NUMBER1;
    mpu_init_struct.subregion_disable    = 0x0;
    mpu_init_struct.instruction_exec     = MPU_INSTRUCTION_EXEC_NOT_PERMIT;
    mpu_init_struct.tex_type             = MPU_TEX_TYPE0;
    mpu_region_config(&mpu_init_struct);
    mpu_region_enable();
    
    /* enable the MPU */
    ARM_MPU_Enable(MPU_MODE_PRIV_DEFAULT);
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
