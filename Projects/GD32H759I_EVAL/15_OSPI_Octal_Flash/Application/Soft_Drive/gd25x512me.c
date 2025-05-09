/*!
    \file    gd25x512me.c
    \brief   OSPI flash gd25x512 driver

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

#include "gd25x512me.h"
#include "gd32h7xx.h"

/*!
    \brief      initialize OSPI/OSPIM and GPIO
    \param[in]  ospi_periph: OSPIx(x=0,1)
    \param[out] ospi_struct: OSPI parameter initialization stuct members of the structure
                             and the member values are shown as below:
                  prescaler: between 0 and 255
                  fifo_threshold: OSPI_FIFO_THRESHOLD_x (x = 1, 2, ..., 31, 32)
                  sample_shift: OSPI_SAMPLE_SHIFTING_NONE, OSPI_SAMPLE_SHIFTING_HALF_CYCLE
                  device_size: OSPI_MESZ_x_BYTES (x = 2, 4, 8, ..., 512, 1024)
                               OSPI_MESZ_x_KBS (x = 2, 4, 8, ..., 512, 1024)
                               OSPI_MESZ_x_MBS (x = 2, 4, 8, ..., 2048, 4096)
                  cs_hightime: OSPI_CS_HIGH_TIME_x_CYCLE (x = 1, 2, ..., 63, 64)
                  memory_type: OSPI_MICRON_MODE, OSPI_MACRONIX_MODE, OSPI_STANDARD_MODE
                               OSPI_MACRONIX_RAM_MODE,
                  wrap_size: OSPI_DIRECT, OSPI_WRAP_16BYTES, OSPI_WRAP_32BYTES
                             OSPI_WRAP_64BYTES, OSPI_WRAP_128BYTES
                  delay_hold_cycle: OSPI_DELAY_HOLD_NONE, OSPI_DELAY_HOLD_QUARTER_CYCLE
    \retval     none
*/
void ospi_flash_init(uint32_t ospi_periph, ospi_parameter_struct *ospi_struct)
{
    /* reset the OSPI and OSPIM peripheral */
    ospi_deinit(ospi_periph);
    ospim_deinit();
    /* enable OSPIM and GPIO clock */
    rcu_periph_clock_enable(RCU_OSPIM);
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOD);

        /* configure OSPIM GPIO pin:
           OSPIM_P0_IO0(PD11) AF9
           OSPIM_P0_IO1(PD12) AF9
           OSPIM_P0_IO2(PA3)  AF6
           OSPIM_P0_IO3(PD13) AF9
           OSPIM_P0_IO4(PD4)  AF10
           OSPIM_P0_IO5(PD5)  AF10
           OSPIM_P0_IO6(PD6)  AF10
           OSPIM_P0_IO7(PD7)  AF10
           OSPIM_P0_CLK(PB2)  AF9
           OSPIM_P0_NCS(PB6)  AF10  */

        gpio_af_set(GPIOB, GPIO_AF_9, GPIO_PIN_2);
        gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
        gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_2);

        gpio_af_set(GPIOA, GPIO_AF_6, GPIO_PIN_3);
        gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
        gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_3);

        gpio_af_set(GPIOD, GPIO_AF_10, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
        gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
        gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

        gpio_af_set(GPIOD, GPIO_AF_9, GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13);
        gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13);
        gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13);

        gpio_af_set(GPIOB, GPIO_AF_10, GPIO_PIN_6);
        gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
        gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_6);

        /* enable SCK, CSN, IO[3:0] and IO[7:4] for OSPIM port0 */
        ospim_port_sck_config(OSPIM_PORT0, OSPIM_PORT_SCK_ENABLE);
        ospim_port_csn_config(OSPIM_PORT0, OSPIM_PORT_CSN_ENABLE);
        ospim_port_io3_0_config(OSPIM_PORT0, OSPIM_IO_LOW_ENABLE);
        ospim_port_io7_4_config(OSPIM_PORT0, OSPIM_IO_HIGH_ENABLE);

    switch(ospi_periph) {
    case OSPI0:
        rcu_periph_clock_enable(RCU_OSPI0);
        /* configure OSPIM port0 */
        ospim_port_sck_source_select(OSPIM_PORT0, OSPIM_SCK_SOURCE_OSPI0_SCK);
        ospim_port_csn_source_select(OSPIM_PORT0, OSPIM_CSN_SOURCE_OSPI0_CSN);
        ospim_port_io3_0_source_select(OSPIM_PORT0, OSPIM_SRCPLIO_OSPI0_IO_LOW);
        ospim_port_io7_4_source_select(OSPIM_PORT0, OSPIM_SRCPHIO_OSPI0_IO_HIGH);
        break;
    case OSPI1:
        rcu_periph_clock_enable(RCU_OSPI1);
        /* configure OSPIM port0 */
        ospim_port_sck_source_select(OSPIM_PORT0, OSPIM_SCK_SOURCE_OSPI1_SCK);
        ospim_port_csn_source_select(OSPIM_PORT0, OSPIM_CSN_SOURCE_OSPI1_CSN);
        ospim_port_io3_0_source_select(OSPIM_PORT0, OSPIM_SRCPLIO_OSPI1_IO_LOW);
        ospim_port_io7_4_source_select(OSPIM_PORT0, OSPIM_SRCPHIO_OSPI1_IO_HIGH);
        break;
    default:
        break;
    }

    /* initialize the parameters of OSPI struct */
    ospi_struct_init(ospi_struct);

    ospi_struct->prescaler = 9U;
    ospi_struct->sample_shift = OSPI_SAMPLE_SHIFTING_NONE;
    ospi_struct->fifo_threshold = OSPI_FIFO_THRESHOLD_5;
    ospi_struct->device_size = OSPI_MESZ_512_MBS;
    ospi_struct->wrap_size = OSPI_DIRECT;
    ospi_struct->cs_hightime          = OSPI_CS_HIGH_TIME_3_CYCLE;
    ospi_struct->memory_type = OSPI_MICRON_MODE;
    ospi_struct->delay_hold_cycle = OSPI_DELAY_HOLD_NONE;

    /* initialize OSPI parameter */
    ospi_init(ospi_periph, ospi_struct);
    /* enable OSPI */
    ospi_enable(ospi_periph);
}

/*!
    \brief      read the flah id
    \param[in]  ospi_periph: OSPIx(x=0,1)
    \param[in]  ospi_struct: OSPI parameter initialization stuct members of the structure
                             and the member values are shown as below:
                  prescaler: between 0 and 255
                  fifo_threshold: OSPI_FIFO_THRESHOLD_x (x = 1, 2, ..., 31, 32)
                  sample_shift: OSPI_SAMPLE_SHIFTING_NONE, OSPI_SAMPLE_SHIFTING_HALF_CYCLE
                  device_size: OSPI_MESZ_x_BYTES (x = 2, 4, 8, ..., 512, 1024)
                             OSPI_MESZ_x_KBS (x = 2, 4, 8, ..., 512, 1024)
                             OSPI_MESZ_x_MBS (x = 2, 4, 8, ..., 2048, 4096)
                  cs_hightime: OSPI_CS_HIGH_TIME_x_CYCLE (x = 1, 2, ..., 63, 64)
                  memory_type: OSPI_MICRON_MODE, OSPI_MACRONIX_MODE, OSPI_STANDARD_MODE
                             OSPI_MACRONIX_RAM_MODE,
                  wrap_size: OSPI_DIRECT, OSPI_WRAP_16BYTES, OSPI_WRAP_32BYTES
                           OSPI_WRAP_64BYTES, OSPI_WRAP_128BYTES
                  delay_hold_cycle: OSPI_DELAY_HOLD_NONE, OSPI_DELAY_HOLD_QUARTER_CYCLE
    \param[in]  mode: flash interface mode
                only one parameter can be selected which is shown as below:
      \arg        SPI_MODE: SPI mode
      \arg        OSPI_MODE: OSPI mode
    \retval     none
*/
uint32_t ospi_flash_read_id(uint32_t ospi_periph, ospi_parameter_struct *ospi_struct, interface_mode mode)
{
    uint8_t temp_id[4];
    ospi_regular_cmd_struct cmd_struct = {0};

    /* initialize read ID command */
    if(SPI_MODE == mode) {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_1_LINE;
        cmd_struct.addr_mode = OSPI_ADDRESS_NONE;
        cmd_struct.data_mode = OSPI_DATA_1_LINE;
        cmd_struct.dummy_cycles = OSPI_DUMYC_CYCLES_0;
    } else {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_8_LINES;
        cmd_struct.addr_mode = OSPI_ADDRESS_NONE;
        cmd_struct.data_mode = OSPI_DATA_8_LINES;
        cmd_struct.dummy_cycles = OSPI_DUMYC_CYCLES_8;
    }
    cmd_struct.operation_type = OSPI_OPTYPE_COMMON_CFG;
    cmd_struct.instruction = GD25X512ME_READ_ID_CMD;
    cmd_struct.ins_size = OSPI_INSTRUCTION_8_BITS;
    cmd_struct.addr_size = OSPI_ADDRESS_24_BITS;
    cmd_struct.addr_dtr_mode = OSPI_ADDRDTR_MODE_DISABLE;
    cmd_struct.alter_bytes_mode = OSPI_ALTERNATE_BYTES_NONE;
    cmd_struct.alter_bytes_size = OSPI_ALTERNATE_BYTES_24_BITS;
    cmd_struct.alter_bytes_dtr_mode = OSPI_ABDTR_MODE_DISABLE;
    cmd_struct.data_dtr_mode = OSPI_DADTR_MODE_DISABLE;
    cmd_struct.nbdata = 4;

    /* send the command */
    ospi_command_config(ospi_periph, ospi_struct, &cmd_struct);

    /* receive data */
    ospi_receive(ospi_periph, temp_id);
    return (temp_id[0] << 24) | (temp_id[1] << 16) | (temp_id[2] << 8) | temp_id[3];
}

/*!
    \brief      enable flash reset
    \param[in]  ospi_periph: OSPIx(x=0,1)
    \param[in]  ospi_struct: OSPI parameter initialization stuct members of the structure
                             and the member values are shown as below:
                  prescaler: between 0 and 255
                  fifo_threshold: OSPI_FIFO_THRESHOLD_x (x = 1, 2, ..., 31, 32)
                  sample_shift: OSPI_SAMPLE_SHIFTING_NONE, OSPI_SAMPLE_SHIFTING_HALF_CYCLE
                  device_size: OSPI_MESZ_x_BYTES (x = 2, 4, 8, ..., 512, 1024)
                             OSPI_MESZ_x_KBS (x = 2, 4, 8, ..., 512, 1024)
                             OSPI_MESZ_x_MBS (x = 2, 4, 8, ..., 2048, 4096)
                  cs_hightime: OSPI_CS_HIGH_TIME_x_CYCLE (x = 1, 2, ..., 63, 64)
                  memory_type: OSPI_MICRON_MODE, OSPI_MACRONIX_MODE, OSPI_STANDARD_MODE
                             OSPI_MACRONIX_RAM_MODE,
                  wrap_size: OSPI_DIRECT, OSPI_WRAP_16BYTES, OSPI_WRAP_32BYTES
                           OSPI_WRAP_64BYTES, OSPI_WRAP_128BYTES
                  delay_hold_cycle: OSPI_DELAY_HOLD_NONE, OSPI_DELAY_HOLD_QUARTER_CYCLE
    \param[in]  mode: flash interface mode
                only one parameter can be selected which is shown as below:
      \arg        SPI_MODE: SPI mode
      \arg        OSPI_MODE: OSPI mode
    \retval     none
*/
void ospi_flash_reset_enable(uint32_t ospi_periph, ospi_parameter_struct *ospi_struct, interface_mode mode)
{
    ospi_regular_cmd_struct cmd_struct = {0};

    /* initialize enable flash reset command */
    if(SPI_MODE == mode) {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_1_LINE;
    } else {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_8_LINES;
    }
    cmd_struct.operation_type = OSPI_OPTYPE_COMMON_CFG;
    cmd_struct.instruction = GD25X512ME_RESET_ENABLE_CMD;
    cmd_struct.ins_size = OSPI_INSTRUCTION_8_BITS;
    cmd_struct.addr_mode = OSPI_ADDRESS_NONE;
    cmd_struct.addr_size = OSPI_ADDRESS_24_BITS;
    cmd_struct.addr_dtr_mode = OSPI_ADDRDTR_MODE_DISABLE;
    cmd_struct.alter_bytes_mode = OSPI_ALTERNATE_BYTES_NONE;
    cmd_struct.alter_bytes_size = OSPI_ALTERNATE_BYTES_24_BITS;
    cmd_struct.alter_bytes_dtr_mode = OSPI_ABDTR_MODE_DISABLE;
    cmd_struct.data_mode = OSPI_DATA_NONE;
    cmd_struct.data_dtr_mode = OSPI_DADTR_MODE_DISABLE;
    cmd_struct.dummy_cycles = OSPI_DUMYC_CYCLES_0;
    cmd_struct.nbdata = 0;

    /* send the command */
    ospi_command_config(ospi_periph, ospi_struct, &cmd_struct);
}

/*!
    \brief      reset the flash
    \param[in]  ospi_periph: OSPIx(x=0,1)
    \param[in]  ospi_struct: OSPI parameter initialization stuct members of the structure
                             and the member values are shown as below:
                  prescaler: between 0 and 255
                  fifo_threshold: OSPI_FIFO_THRESHOLD_x (x = 1, 2, ..., 31, 32)
                  sample_shift: OSPI_SAMPLE_SHIFTING_NONE, OSPI_SAMPLE_SHIFTING_HALF_CYCLE
                  device_size: OSPI_MESZ_x_BYTES (x = 2, 4, 8, ..., 512, 1024)
                             OSPI_MESZ_x_KBS (x = 2, 4, 8, ..., 512, 1024)
                             OSPI_MESZ_x_MBS (x = 2, 4, 8, ..., 2048, 4096)
                  cs_hightime: OSPI_CS_HIGH_TIME_x_CYCLE (x = 1, 2, ..., 63, 64)
                  memory_type: OSPI_MICRON_MODE, OSPI_MACRONIX_MODE, OSPI_STANDARD_MODE
                             OSPI_MACRONIX_RAM_MODE,
                  wrap_size: OSPI_DIRECT, OSPI_WRAP_16BYTES, OSPI_WRAP_32BYTES
                           OSPI_WRAP_64BYTES, OSPI_WRAP_128BYTES
                  delay_hold_cycle: OSPI_DELAY_HOLD_NONE, OSPI_DELAY_HOLD_QUARTER_CYCLE
    \param[in]  mode: flash interface mode
                only one parameter can be selected which is shown as below:
      \arg        SPI_MODE: SPI mode
      \arg        OSPI_MODE: OSPI mode
    \retval     none
*/
void ospi_flash_reset_memory(uint32_t ospi_periph, ospi_parameter_struct *ospi_struct, interface_mode mode)
{
    ospi_regular_cmd_struct cmd_struct = {0};

    /* initialize flash reset command */
    if(SPI_MODE == mode) {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_1_LINE;
    } else {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_8_LINES;
    }
    cmd_struct.operation_type = OSPI_OPTYPE_COMMON_CFG;
    cmd_struct.instruction = GD25X512ME_RESET_MEMORY_CMD;
    cmd_struct.ins_size = OSPI_INSTRUCTION_8_BITS;
    cmd_struct.addr_mode = OSPI_ADDRESS_NONE;
    cmd_struct.addr_size = OSPI_ADDRESS_24_BITS;
    cmd_struct.addr_dtr_mode = OSPI_ADDRDTR_MODE_DISABLE;
    cmd_struct.alter_bytes_mode = OSPI_ALTERNATE_BYTES_NONE;
    cmd_struct.alter_bytes_size = OSPI_ALTERNATE_BYTES_24_BITS;
    cmd_struct.alter_bytes_dtr_mode = OSPI_ABDTR_MODE_DISABLE;
    cmd_struct.data_mode = OSPI_DATA_NONE;
    cmd_struct.data_dtr_mode = OSPI_DADTR_MODE_DISABLE;
    cmd_struct.dummy_cycles = OSPI_DUMYC_CYCLES_0;
    cmd_struct.nbdata = 0;

    /* send the command */
    ospi_command_config(ospi_periph, ospi_struct, &cmd_struct);
}

/*!
    \brief      enable flash write
    \param[in]  ospi_periph: OSPIx(x=0,1)
    \param[in]  ospi_struct: OSPI parameter initialization stuct members of the structure
                             and the member values are shown as below:
                  prescaler: between 0 and 255
                  fifo_threshold: OSPI_FIFO_THRESHOLD_x (x = 1, 2, ..., 31, 32)
                  sample_shift: OSPI_SAMPLE_SHIFTING_NONE, OSPI_SAMPLE_SHIFTING_HALF_CYCLE
                  device_size: OSPI_MESZ_x_BYTES (x = 2, 4, 8, ..., 512, 1024)
                             OSPI_MESZ_x_KBS (x = 2, 4, 8, ..., 512, 1024)
                             OSPI_MESZ_x_MBS (x = 2, 4, 8, ..., 2048, 4096)
                  cs_hightime: OSPI_CS_HIGH_TIME_x_CYCLE (x = 1, 2, ..., 63, 64)
                  memory_type: OSPI_MICRON_MODE, OSPI_MACRONIX_MODE, OSPI_STANDARD_MODE
                             OSPI_MACRONIX_RAM_MODE,
                  wrap_size: OSPI_DIRECT, OSPI_WRAP_16BYTES, OSPI_WRAP_32BYTES
                           OSPI_WRAP_64BYTES, OSPI_WRAP_128BYTES
                  delay_hold_cycle: OSPI_DELAY_HOLD_NONE, OSPI_DELAY_HOLD_QUARTER_CYCLE
    \param[in]  mode: flash interface mode
                only one parameter can be selected which is shown as below:
      \arg        SPI_MODE: SPI mode
      \arg        OSPI_MODE: OSPI mode
    \retval     none
*/
void ospi_flash_write_enbale(uint32_t ospi_periph, ospi_parameter_struct *ospi_struct, interface_mode mode)
{
    ospi_autopolling_struct autopl_cfg_struct = {0};
    ospi_regular_cmd_struct cmd_struct = {0};

    /* initialize write enable command */
    if(SPI_MODE == mode) {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_1_LINE;
    } else {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_8_LINES;
    }
    cmd_struct.operation_type = OSPI_OPTYPE_COMMON_CFG;
    cmd_struct.instruction = GD25X512ME_WRITE_ENABLE_CMD;
    cmd_struct.ins_size = OSPI_INSTRUCTION_8_BITS;
    cmd_struct.addr_mode = OSPI_ADDRESS_NONE;
    cmd_struct.addr_size = OSPI_ADDRESS_24_BITS;
    cmd_struct.addr_dtr_mode = OSPI_ADDRDTR_MODE_DISABLE;
    cmd_struct.alter_bytes_mode = OSPI_ALTERNATE_BYTES_NONE;
    cmd_struct.alter_bytes_size = OSPI_ALTERNATE_BYTES_24_BITS;
    cmd_struct.alter_bytes_dtr_mode = OSPI_ABDTR_MODE_DISABLE;
    cmd_struct.data_mode = OSPI_DATA_NONE;
    cmd_struct.data_dtr_mode = OSPI_DADTR_MODE_DISABLE;
    cmd_struct.dummy_cycles = OSPI_DUMYC_CYCLES_0;
    cmd_struct.nbdata = 0;

    /* send the command */
    ospi_command_config(ospi_periph, ospi_struct, &cmd_struct);

    /* configure automatic polling mode to wait for write enabling */
    if(SPI_MODE == mode) {
        cmd_struct.data_mode = OSPI_DATA_1_LINE;
        cmd_struct.dummy_cycles = OSPI_DUMYC_CYCLES_0;
    } else {
        cmd_struct.data_mode = OSPI_DATA_8_LINES;
        cmd_struct.dummy_cycles = OSPI_DUMYC_CYCLES_8;
    }
    cmd_struct.instruction = GD25X512ME_READ_STATUS_REG_CMD;
    cmd_struct.addr_dtr_mode = OSPI_ADDRDTR_MODE_DISABLE;
    cmd_struct.nbdata = 1;

    /* send the command */
    ospi_command_config(ospi_periph, ospi_struct, &cmd_struct);

    autopl_cfg_struct.match = 2U;
    autopl_cfg_struct.mask  = 2U;
    autopl_cfg_struct.match_mode = OSPI_MATCH_MODE_AND;
    autopl_cfg_struct.interval = GD25X512ME_AUTOPOLLING_INTERVAL_TIME;
    autopl_cfg_struct.automatic_stop = OSPI_AUTOMATIC_STOP_MATCH;
    ospi_autopolling_mode(ospi_periph, ospi_struct, &autopl_cfg_struct);
}

/*!
    \brief      write flash volatile configuration register
    \param[in]  ospi_periph: OSPIx(x=0,1)
    \param[in]  ospi_struct: OSPI parameter initialization stuct members of the structure
                             and the member values are shown as below:
                  prescaler: between 0 and 255
                  fifo_threshold: OSPI_FIFO_THRESHOLD_x (x = 1, 2, ..., 31, 32)
                  sample_shift: OSPI_SAMPLE_SHIFTING_NONE, OSPI_SAMPLE_SHIFTING_HALF_CYCLE
                  device_size: OSPI_MESZ_x_BYTES (x = 2, 4, 8, ..., 512, 1024)
                             OSPI_MESZ_x_KBS (x = 2, 4, 8, ..., 512, 1024)
                             OSPI_MESZ_x_MBS (x = 2, 4, 8, ..., 2048, 4096)
                  cs_hightime: OSPI_CS_HIGH_TIME_x_CYCLE (x = 1, 2, ..., 63, 64)
                  memory_type: OSPI_MICRON_MODE, OSPI_MACRONIX_MODE, OSPI_STANDARD_MODE
                             OSPI_MACRONIX_RAM_MODE,
                  wrap_size: OSPI_DIRECT, OSPI_WRAP_16BYTES, OSPI_WRAP_32BYTES
                           OSPI_WRAP_64BYTES, OSPI_WRAP_128BYTES
                  delay_hold_cycle: OSPI_DELAY_HOLD_NONE, OSPI_DELAY_HOLD_QUARTER_CYCLE
    \param[in]  mode: flash interface mode
                only one parameter can be selected which is shown as below:
      \arg        SPI_MODE: SPI mode
      \arg        OSPI_MODE: OSPI mode
    \param[in]  addr_size: the size of address
    \param[in]  value: the value of transmit
    \retval     none
*/
void ospi_flash_write_volatilecfg_register(uint32_t ospi_periph, ospi_parameter_struct *ospi_struct, interface_mode mode, addr_size addr_size,
        uint32_t addr, uint8_t value)
{
    ospi_regular_cmd_struct cmd_struct = {0};

    /* initialize write enable for volatile status register command */
    if(SPI_MODE == mode) {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_1_LINE;
    } else {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_8_LINES;
    }
    cmd_struct.operation_type = OSPI_OPTYPE_COMMON_CFG;
    cmd_struct.instruction = GD25X512ME_WRITE_ENABLE_VOLATILE_STATUS_CFG_CMD;
    cmd_struct.ins_size = OSPI_INSTRUCTION_8_BITS;
    cmd_struct.addr_mode = OSPI_ADDRESS_NONE;
    cmd_struct.addr_size = OSPI_ADDRESS_24_BITS;
    cmd_struct.addr_dtr_mode = OSPI_ADDRDTR_MODE_DISABLE;
    cmd_struct.alter_bytes_mode = OSPI_ALTERNATE_BYTES_NONE;
    cmd_struct.alter_bytes_size = OSPI_ALTERNATE_BYTES_24_BITS;
    cmd_struct.alter_bytes_dtr_mode = OSPI_ABDTR_MODE_DISABLE;
    cmd_struct.data_mode = OSPI_DATA_NONE;
    cmd_struct.data_dtr_mode = OSPI_DADTR_MODE_DISABLE;
    cmd_struct.dummy_cycles = OSPI_DUMYC_CYCLES_0;
    cmd_struct.nbdata = 0;

    /* send the command */
    ospi_command_config(ospi_periph, ospi_struct, &cmd_struct);

    /* initialize write volatile configuration register command */
    if(SPI_MODE == mode) {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_1_LINE;
        cmd_struct.addr_mode = OSPI_ADDRESS_1_LINE;
        cmd_struct.data_mode = OSPI_DATA_1_LINE;
    } else {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_8_LINES;
        cmd_struct.addr_mode = OSPI_ADDRESS_8_LINES;
        cmd_struct.data_mode = OSPI_DATA_8_LINES;
    }
    if(GD25X512ME_3BYTES_SIZE == addr_size) {
        cmd_struct.addr_size = OSPI_ADDRESS_24_BITS;
    } else {
        cmd_struct.addr_size = OSPI_ADDRESS_32_BITS;
    }
    cmd_struct.operation_type = OSPI_OPTYPE_COMMON_CFG;
    cmd_struct.instruction = GD25X512ME_WRITE_VOLATILE_CFG_REG_CMD;
    cmd_struct.ins_size = OSPI_INSTRUCTION_8_BITS;
    cmd_struct.address = addr;
    cmd_struct.addr_dtr_mode = OSPI_ADDRDTR_MODE_DISABLE;
    cmd_struct.alter_bytes_mode = OSPI_ALTERNATE_BYTES_NONE;
    cmd_struct.alter_bytes_size = OSPI_ALTERNATE_BYTES_24_BITS;
    cmd_struct.alter_bytes_dtr_mode = OSPI_ABDTR_MODE_DISABLE;
    cmd_struct.data_dtr_mode = OSPI_DADTR_MODE_DISABLE;
    cmd_struct.dummy_cycles = OSPI_DUMYC_CYCLES_0;
    cmd_struct.nbdata = 1;

    /* send the command */
    ospi_command_config(ospi_periph, ospi_struct, &cmd_struct);
    ospi_transmit(ospi_periph, &value);
}

/*!
    \brief      polling WIP(Write In Progress) bit become to 0
    \param[in]  ospi_periph: OSPIx(x=0,1)
    \param[in]  ospi_struct: OSPI parameter initialization stuct members of the structure
                             and the member values are shown as below:
                  prescaler: between 0 and 255
                  fifo_threshold: OSPI_FIFO_THRESHOLD_x (x = 1, 2, ..., 31, 32)
                  sample_shift: OSPI_SAMPLE_SHIFTING_NONE, OSPI_SAMPLE_SHIFTING_HALF_CYCLE
                  device_size: OSPI_MESZ_x_BYTES (x = 2, 4, 8, ..., 512, 1024)
                             OSPI_MESZ_x_KBS (x = 2, 4, 8, ..., 512, 1024)
                             OSPI_MESZ_x_MBS (x = 2, 4, 8, ..., 2048, 4096)
                  cs_hightime: OSPI_CS_HIGH_TIME_x_CYCLE (x = 1, 2, ..., 63, 64)
                  memory_type: OSPI_MICRON_MODE, OSPI_MACRONIX_MODE, OSPI_STANDARD_MODE
                             OSPI_MACRONIX_RAM_MODE,
                  wrap_size: OSPI_DIRECT, OSPI_WRAP_16BYTES, OSPI_WRAP_32BYTES
                           OSPI_WRAP_64BYTES, OSPI_WRAP_128BYTES
                  delay_hold_cycle: OSPI_DELAY_HOLD_NONE, OSPI_DELAY_HOLD_QUARTER_CYCLE
    \param[in]  mode: flash interface mode
                only one parameter can be selected which is shown as below:
      \arg        SPI_MODE: SPI mode
      \arg        OSPI_MODE: OSPI mode
    \retval     none
*/
void ospi_flash_autopolling_mem_ready(uint32_t ospi_periph, ospi_parameter_struct *ospi_struct, interface_mode mode)
{
    ospi_autopolling_struct autopl_cfg_struct = {0};
    ospi_regular_cmd_struct cmd_struct = {0};

    /* initialize read status register command */
    if(SPI_MODE == mode) {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_1_LINE;
        cmd_struct.addr_mode = OSPI_ADDRESS_NONE;
        cmd_struct.data_mode = OSPI_DATA_1_LINE;
        cmd_struct.dummy_cycles = OSPI_DUMYC_CYCLES_0;
    } else {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_8_LINES;
        cmd_struct.addr_mode = OSPI_ADDRESS_NONE;
        cmd_struct.data_mode = OSPI_DATA_8_LINES;
        cmd_struct.dummy_cycles = OSPI_DUMYC_CYCLES_8;
    }
    cmd_struct.operation_type = OSPI_OPTYPE_COMMON_CFG;
    cmd_struct.ins_size = OSPI_INSTRUCTION_8_BITS;
    cmd_struct.instruction = GD25X512ME_READ_STATUS_REG_CMD;
    cmd_struct.addr_dtr_mode = OSPI_ADDRDTR_MODE_DISABLE;
    cmd_struct.addr_size = OSPI_ADDRESS_24_BITS;
    cmd_struct.address = 0U;
    cmd_struct.alter_bytes_mode = OSPI_ALTERNATE_BYTES_NONE;
    cmd_struct.alter_bytes_size = OSPI_ALTERNATE_BYTES_24_BITS;
    cmd_struct.alter_bytes_dtr_mode = OSPI_ABDTR_MODE_DISABLE;
    cmd_struct.data_dtr_mode = OSPI_DADTR_MODE_DISABLE;
    cmd_struct.nbdata = 1;

    /* send the command */
    ospi_command_config(ospi_periph, ospi_struct, &cmd_struct);

    /* configure the OSPI automatic polling mode */
    autopl_cfg_struct.match = 0U;
    autopl_cfg_struct.mask  = GD25X512ME_SR_WIP;
    autopl_cfg_struct.match_mode = OSPI_MATCH_MODE_AND;
    autopl_cfg_struct.interval = GD25X512ME_AUTOPOLLING_INTERVAL_TIME;
    autopl_cfg_struct.automatic_stop = OSPI_AUTOMATIC_STOP_MATCH;
    ospi_autopolling_mode(ospi_periph, ospi_struct, &autopl_cfg_struct);
}

/*!
    \brief      erase the specified block of the flash
    \param[in]  ospi_periph: OSPIx(x=0,1)
    \param[in]  ospi_struct: OSPI parameter initialization stuct members of the structure
                             and the member values are shown as below:
                  prescaler: between 0 and 255
                  fifo_threshold: OSPI_FIFO_THRESHOLD_x (x = 1, 2, ..., 31, 32)
                  sample_shift: OSPI_SAMPLE_SHIFTING_NONE, OSPI_SAMPLE_SHIFTING_HALF_CYCLE
                  device_size: OSPI_MESZ_x_BYTES (x = 2, 4, 8, ..., 512, 1024)
                             OSPI_MESZ_x_KBS (x = 2, 4, 8, ..., 512, 1024)
                             OSPI_MESZ_x_MBS (x = 2, 4, 8, ..., 2048, 4096)
                  cs_hightime: OSPI_CS_HIGH_TIME_x_CYCLE (x = 1, 2, ..., 63, 64)
                  memory_type: OSPI_MICRON_MODE, OSPI_MACRONIX_MODE, OSPI_STANDARD_MODE
                             OSPI_MACRONIX_RAM_MODE,
                  wrap_size: OSPI_DIRECT, OSPI_WRAP_16BYTES, OSPI_WRAP_32BYTES
                           OSPI_WRAP_64BYTES, OSPI_WRAP_128BYTES
                  delay_hold_cycle: OSPI_DELAY_HOLD_NONE, OSPI_DELAY_HOLD_QUARTER_CYCLE
    \param[in]  mode: flash interface mode
                only one parameter can be selected which is shown as below:
      \arg        SPI_MODE: SPI mode
      \arg        OSPI_MODE: OSPI mode
    \param[in]  addr_size: the size of address
    \param[in]  address: address of erase
    \param[in]  block_size: block size to erase
    \retval     none
*/
void ospi_flash_block_erase(uint32_t ospi_periph, ospi_parameter_struct *ospi_struct, interface_mode mode, addr_size addr_size, uint32_t addr,
                            erase_size block_size)
{
    ospi_regular_cmd_struct cmd_struct = {0};

    /* initialize block erase command */
    if(SPI_MODE == mode) {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_1_LINE;
        cmd_struct.addr_mode = OSPI_ADDRESS_1_LINE;
    } else {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_8_LINES;
        cmd_struct.addr_mode = OSPI_ADDRESS_8_LINES;
    }
    if(GD25X512ME_ERASE_64K == block_size) {
        if(GD25X512ME_3BYTES_SIZE == addr_size) {
            cmd_struct.instruction = GD25X512ME_BLOCK_ERASE_64K_CMD;
        } else {
            cmd_struct.instruction = GD25X512ME_4_BYTE_BLOCK_ERASE_64K_CMD;
        }
    } else {
        if(GD25X512ME_3BYTES_SIZE == addr_size) {
            cmd_struct.instruction = GD25X512ME_SECTOR_ERASE_4K_CMD;
        } else {
            cmd_struct.instruction = GD25X512ME_4_BYTE_SECTOR_ERASE_4K_CMD;
        }
    }
    cmd_struct.operation_type = OSPI_OPTYPE_COMMON_CFG;
    cmd_struct.ins_size = OSPI_INSTRUCTION_8_BITS;
    cmd_struct.addr_size = OSPI_ADDRESS_24_BITS;
    cmd_struct.address = addr;
    cmd_struct.alter_bytes_mode = OSPI_ALTERNATE_BYTES_NONE;
    cmd_struct.alter_bytes_size = OSPI_ALTERNATE_BYTES_24_BITS;
    cmd_struct.alter_bytes_dtr_mode = OSPI_ABDTR_MODE_DISABLE;
    cmd_struct.data_mode = OSPI_DATA_NONE;
    cmd_struct.data_dtr_mode = OSPI_DADTR_MODE_DISABLE;
    cmd_struct.dummy_cycles = OSPI_DUMYC_CYCLES_0;
    cmd_struct.nbdata = 0;

    /* send the command */
    ospi_command_config(ospi_periph, ospi_struct, &cmd_struct);
}

/*!
    \brief      erase the the entire flash
    \param[in]  ospi_periph: OSPIx(x=0,1)
    \param[in]  ospi_struct: OSPI parameter initialization stuct members of the structure
                             and the member values are shown as below:
                  prescaler: between 0 and 255
                  fifo_threshold: OSPI_FIFO_THRESHOLD_x (x = 1, 2, ..., 31, 32)
                  sample_shift: OSPI_SAMPLE_SHIFTING_NONE, OSPI_SAMPLE_SHIFTING_HALF_CYCLE
                  device_size: OSPI_MESZ_x_BYTES (x = 2, 4, 8, ..., 512, 1024)
                             OSPI_MESZ_x_KBS (x = 2, 4, 8, ..., 512, 1024)
                             OSPI_MESZ_x_MBS (x = 2, 4, 8, ..., 2048, 4096)
                  cs_hightime: OSPI_CS_HIGH_TIME_x_CYCLE (x = 1, 2, ..., 63, 64)
                  memory_type: OSPI_MICRON_MODE, OSPI_MACRONIX_MODE, OSPI_STANDARD_MODE
                             OSPI_MACRONIX_RAM_MODE,
                  wrap_size: OSPI_DIRECT, OSPI_WRAP_16BYTES, OSPI_WRAP_32BYTES
                           OSPI_WRAP_64BYTES, OSPI_WRAP_128BYTES
                  delay_hold_cycle: OSPI_DELAY_HOLD_NONE, OSPI_DELAY_HOLD_QUARTER_CYCLE
    \param[in]  mode: flash interface mode
                only one parameter can be selected which is shown as below:
      \arg        SPI_MODE: SPI mode
      \arg        OSPI_MODE: OSPI mode
    \retval     none
*/
void ospi_flash_chip_erase(uint32_t ospi_periph, ospi_parameter_struct *ospi_struct, interface_mode mode)
{
    ospi_regular_cmd_struct cmd_struct = {0};

    /* initialize block erase command */
    if(SPI_MODE == mode) {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_1_LINE;
    } else {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_8_LINES;
    }

    cmd_struct.operation_type = OSPI_OPTYPE_COMMON_CFG;
    cmd_struct.ins_size = OSPI_INSTRUCTION_8_BITS;
    cmd_struct.instruction = GD25X512ME_CHIP_ERASE_CMD;
    cmd_struct.addr_mode = OSPI_ADDRESS_NONE;
    cmd_struct.alter_bytes_mode = OSPI_ALTERNATE_BYTES_NONE;
    cmd_struct.alter_bytes_size = OSPI_ALTERNATE_BYTES_24_BITS;
    cmd_struct.alter_bytes_dtr_mode = OSPI_ABDTR_MODE_DISABLE;
    cmd_struct.data_mode = OSPI_DATA_NONE;
    cmd_struct.data_dtr_mode = OSPI_DADTR_MODE_DISABLE;
    cmd_struct.dummy_cycles = OSPI_DUMYC_CYCLES_0;
    cmd_struct.nbdata = 0;

    /* send the command */
    ospi_command_config(ospi_periph, ospi_struct, &cmd_struct);
}

/*!
    \brief      write data to the flash
    \param[in]  ospi_periph: OSPIx(x=0,1)
    \param[in]  ospi_struct: OSPI parameter initialization stuct members of the structure
                             and the member values are shown as below:
                  prescaler: between 0 and 255
                  fifo_threshold: OSPI_FIFO_THRESHOLD_x (x = 1, 2, ..., 31, 32)
                  sample_shift: OSPI_SAMPLE_SHIFTING_NONE, OSPI_SAMPLE_SHIFTING_HALF_CYCLE
                  device_size: OSPI_MESZ_x_BYTES (x = 2, 4, 8, ..., 512, 1024)
                             OSPI_MESZ_x_KBS (x = 2, 4, 8, ..., 512, 1024)
                             OSPI_MESZ_x_MBS (x = 2, 4, 8, ..., 2048, 4096)
                  cs_hightime: OSPI_CS_HIGH_TIME_x_CYCLE (x = 1, 2, ..., 63, 64)
                  memory_type: OSPI_MICRON_MODE, OSPI_MACRONIX_MODE, OSPI_STANDARD_MODE
                             OSPI_MACRONIX_RAM_MODE,
                  wrap_size: OSPI_DIRECT, OSPI_WRAP_16BYTES, OSPI_WRAP_32BYTES
                           OSPI_WRAP_64BYTES, OSPI_WRAP_128BYTES
                  delay_hold_cycle: OSPI_DELAY_HOLD_NONE, OSPI_DELAY_HOLD_QUARTER_CYCLE
    \param[in]  mode: flash interface mode
                only one parameter can be selected which is shown as below:
      \arg        SPI_MODE: SPI mode
      \arg        OSPI_MODE: OSPI mode
    \param[in]  addr_size: the size of address
    \param[in]  pdata: pointer to data to be written
    \param[in]  addr: write start address
    \param[in]  data_size: size of data to write
    \retval     none
*/
void ospi_flash_page_program(uint32_t ospi_periph, ospi_parameter_struct *ospi_struct, interface_mode mode, addr_size addr_size, uint8_t *pdata,
                             uint32_t addr, uint32_t data_size)
{
    ospi_regular_cmd_struct cmd_struct = {0};

    /* initialize program command */
    if(SPI_MODE == mode) {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_1_LINE;
        cmd_struct.addr_mode = OSPI_ADDRESS_1_LINE;
        cmd_struct.data_mode = OSPI_DATA_1_LINE;
    } else {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_8_LINES;
        cmd_struct.addr_mode = OSPI_ADDRESS_8_LINES;
        cmd_struct.data_mode = OSPI_DATA_8_LINES;
    }
    if(GD25X512ME_3BYTES_SIZE == addr_size) {
        cmd_struct.instruction = GD25X512ME_PAGE_PROG_CMD;
        cmd_struct.addr_size = OSPI_ADDRESS_24_BITS;
    } else {
        cmd_struct.instruction = GD25X512ME_4_BYTE_PAGE_PROG_CMD;
        cmd_struct.addr_size = OSPI_ADDRESS_32_BITS;
    }
    cmd_struct.operation_type = OSPI_OPTYPE_COMMON_CFG;
    cmd_struct.ins_size = OSPI_INSTRUCTION_8_BITS;
    cmd_struct.addr_dtr_mode = OSPI_ADDRDTR_MODE_DISABLE;
    cmd_struct.address = addr;
    cmd_struct.alter_bytes_mode = OSPI_ALTERNATE_BYTES_NONE;
    cmd_struct.alter_bytes_size = OSPI_ALTERNATE_BYTES_24_BITS;
    cmd_struct.alter_bytes_dtr_mode = OSPI_ABDTR_MODE_DISABLE;
    cmd_struct.data_dtr_mode = OSPI_DADTR_MODE_DISABLE;
    cmd_struct.nbdata = data_size;
    cmd_struct.dummy_cycles = OSPI_DUMYC_CYCLES_0;

    /* send the command */
    ospi_command_config(ospi_periph, ospi_struct, &cmd_struct);
    /* transmission of the data */
    ospi_transmit(ospi_periph, pdata);
}

/*!
    \brief      read data from the flash
    \param[in]  ospi_periph: OSPIx(x=0,1)
    \param[in]  ospi_struct: OSPI parameter initialization stuct members of the structure
                             and the member values are shown as below:
                  prescaler: between 0 and 255
                  fifo_threshold: OSPI_FIFO_THRESHOLD_x (x = 1, 2, ..., 31, 32)
                  sample_shift: OSPI_SAMPLE_SHIFTING_NONE, OSPI_SAMPLE_SHIFTING_HALF_CYCLE
                  device_size: OSPI_MESZ_x_BYTES (x = 2, 4, 8, ..., 512, 1024)
                             OSPI_MESZ_x_KBS (x = 2, 4, 8, ..., 512, 1024)
                             OSPI_MESZ_x_MBS (x = 2, 4, 8, ..., 2048, 4096)
                  cs_hightime: OSPI_CS_HIGH_TIME_x_CYCLE (x = 1, 2, ..., 63, 64)
                  memory_type: OSPI_MICRON_MODE, OSPI_MACRONIX_MODE, OSPI_STANDARD_MODE
                             OSPI_MACRONIX_RAM_MODE,
                  wrap_size: OSPI_DIRECT, OSPI_WRAP_16BYTES, OSPI_WRAP_32BYTES
                           OSPI_WRAP_64BYTES, OSPI_WRAP_128BYTES
                  delay_hold_cycle: OSPI_DELAY_HOLD_NONE, OSPI_DELAY_HOLD_QUARTER_CYCLE
    \param[in]  mode: flash interface mode
                only one parameter can be selected which is shown as below:
      \arg        SPI_MODE: SPI mode
      \arg        OSPI_MODE: OSPI mode
    \param[in]  addr_size: the size of address
    \param[in]  pdata: pointer to data to be read
    \param[in]  addr: read start address
    \param[in]  data_size: size of data to read
    \retval     none
*/
void ospi_flash_read(uint32_t ospi_periph, ospi_parameter_struct *ospi_struct, interface_mode mode, addr_size addr_size, uint8_t *pdata,
                     uint32_t addr, uint32_t data_size)
{
    ospi_regular_cmd_struct cmd_struct = {0};

    /* initialize read command */
    if(SPI_MODE == mode) {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_1_LINE;
        cmd_struct.addr_mode = OSPI_ADDRESS_1_LINE;
        cmd_struct.data_mode = OSPI_DATA_1_LINE;
        cmd_struct.dummy_cycles = OSPI_DUMYC_CYCLES_8;
    } else {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_8_LINES;
        cmd_struct.addr_mode = OSPI_ADDRESS_8_LINES;
        cmd_struct.data_mode = OSPI_DATA_8_LINES;
        cmd_struct.dummy_cycles = OSPI_DUMYC_CYCLES_16;
    }
    if(GD25X512ME_3BYTES_SIZE == addr_size) {
        cmd_struct.instruction = GD25X512ME_FAST_READ_CMD;
        cmd_struct.addr_size = OSPI_ADDRESS_24_BITS;
    } else {
        cmd_struct.instruction = GD25X512ME_4_BYTE_ADDR_FAST_READ_CMD;
        cmd_struct.addr_size = OSPI_ADDRESS_32_BITS;
    }
    cmd_struct.operation_type = OSPI_OPTYPE_COMMON_CFG;
    cmd_struct.ins_size = OSPI_INSTRUCTION_8_BITS;
    cmd_struct.addr_dtr_mode = OSPI_ADDRDTR_MODE_DISABLE;
    cmd_struct.address = addr;
    cmd_struct.alter_bytes_mode = OSPI_ALTERNATE_BYTES_NONE;
    cmd_struct.alter_bytes_size = OSPI_ALTERNATE_BYTES_24_BITS;
    cmd_struct.alter_bytes_dtr_mode = OSPI_ABDTR_MODE_DISABLE;
    cmd_struct.data_dtr_mode = OSPI_DADTR_MODE_DISABLE;
    cmd_struct.nbdata = data_size;

    /* send the command */
    ospi_command_config(ospi_periph, ospi_struct, &cmd_struct);

    /* reception of the data */
    ospi_receive(ospi_periph, pdata);
}

int32_t ospi_read_status_register(uint32_t ospi_periph, ospi_parameter_struct *ospi_struct, interface_mode mode)
{
    uint8_t status;
    ospi_regular_cmd_struct cmd_struct = {0};
     
    /* initialize read command */
    if(SPI_MODE == mode) {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_1_LINE;
        cmd_struct.data_mode = OSPI_DATA_1_LINE;
        cmd_struct.dummy_cycles = OSPI_DUMYC_CYCLES_0;
    } else {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_8_LINES;
        cmd_struct.data_mode = OSPI_DATA_8_LINES;
        cmd_struct.dummy_cycles = OSPI_DUMYC_CYCLES_8;
    }
    
    cmd_struct.operation_type = OSPI_OPTYPE_COMMON_CFG;
    cmd_struct.ins_size = OSPI_INSTRUCTION_8_BITS;
    cmd_struct.instruction = GD25X512ME_READ_STATUS_REG_CMD;
    cmd_struct.addr_mode = OSPI_ADDRESS_NONE;
    cmd_struct.alter_bytes_mode = OSPI_ALTERNATE_BYTES_NONE;
    cmd_struct.data_dtr_mode = OSPI_DADTR_MODE_DISABLE;
    cmd_struct.nbdata = 1;
    
    /* send the command */
    ospi_command_config(ospi_periph, ospi_struct, &cmd_struct);
   
    /* reception of the data */
    ospi_receive(ospi_periph, &status);
    
    return status;
}

/*!
    \brief      enable memory mapped mode
    \param[in]  ospi_periph: OSPIx(x=0,1)
    \param[in]  ospi_struct: OSPI parameter initialization stuct members of the structure
                             and the member values are shown as below:
                  prescaler: between 0 and 255
                  fifo_threshold: OSPI_FIFO_THRESHOLD_x (x = 1, 2, ..., 31, 32)
                  sample_shift: OSPI_SAMPLE_SHIFTING_NONE, OSPI_SAMPLE_SHIFTING_HALF_CYCLE
                  device_size: OSPI_MESZ_x_BYTES (x = 2, 4, 8, ..., 512, 1024)
                             OSPI_MESZ_x_KBS (x = 2, 4, 8, ..., 512, 1024)
                             OSPI_MESZ_x_MBS (x = 2, 4, 8, ..., 2048, 4096)
                  cs_hightime: OSPI_CS_HIGH_TIME_x_CYCLE (x = 1, 2, ..., 63, 64)
                  memory_type: OSPI_MICRON_MODE, OSPI_MACRONIX_MODE, OSPI_STANDARD_MODE
                             OSPI_MACRONIX_RAM_MODE,
                  wrap_size: OSPI_DIRECT, OSPI_WRAP_16BYTES, OSPI_WRAP_32BYTES
                           OSPI_WRAP_64BYTES, OSPI_WRAP_128BYTES
                  delay_hold_cycle: OSPI_DELAY_HOLD_NONE, OSPI_DELAY_HOLD_QUARTER_CYCLE
    \param[in]  mode: flash interface mode
                only one parameter can be selected which is shown as below:
      \arg        SPI_MODE: SPI mode
      \arg        OSPI_MODE: OSPI mode
    \param[in]  addr_size: the size of address
    \retval     none
*/
void ospi_flash_memory_map_mode_enable(uint32_t ospi_periph, ospi_parameter_struct *ospi_struct, interface_mode mode, addr_size addr_size)
{
    ospi_regular_cmd_struct cmd_struct = {0};

    /* initialize read command */
    if(SPI_MODE == mode) {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_1_LINE;
    } else {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_8_LINES;
    }
    if(GD25X512ME_3BYTES_SIZE == addr_size) {
        cmd_struct.instruction = GD25X512ME_OCTAL_IO_FAST_READ_CMD;
        cmd_struct.addr_size = OSPI_ADDRESS_24_BITS;
    } else {
        cmd_struct.instruction = GD25X512ME_4_BYTE_ADDR_OCTAL_IO_FAST_READ_CMD;
        cmd_struct.addr_size = OSPI_ADDRESS_32_BITS;
    }
    cmd_struct.operation_type = OSPI_OPTYPE_READ_CFG;
    cmd_struct.ins_size = OSPI_INSTRUCTION_8_BITS;
    cmd_struct.addr_mode = OSPI_ADDRESS_8_LINES;
    cmd_struct.addr_dtr_mode = OSPI_ADDRDTR_MODE_DISABLE;
    cmd_struct.alter_bytes_mode = OSPI_ALTERNATE_BYTES_NONE;
    cmd_struct.alter_bytes_size = OSPI_ALTERNATE_BYTES_24_BITS;
    cmd_struct.alter_bytes_dtr_mode = OSPI_ABDTR_MODE_DISABLE;
    cmd_struct.data_mode = OSPI_DATA_8_LINES;
    cmd_struct.data_dtr_mode = OSPI_DADTR_MODE_DISABLE;
    cmd_struct.dummy_cycles = OSPI_DUMYC_CYCLES_16;

    /* send the command */
    ospi_command_config(ospi_periph, ospi_struct, &cmd_struct);

    /* initialize program command */
    cmd_struct.operation_type = OSPI_OPTYPE_WRITE_CFG;
    cmd_struct.dummy_cycles = OSPI_DUMYC_CYCLES_0;

    if(GD25X512ME_3BYTES_SIZE == addr_size) {
        cmd_struct.instruction = GD25X512ME_EXT_OCTAL_PAGE_PROG_CMD;
    } else {
        cmd_struct.instruction = GD25X512ME_4_BYTE_EXT_OCTAL_PAGE_PROG_CMD;
    }

    /* send the command */
    ospi_command_config(ospi_periph, ospi_struct, &cmd_struct);

    /* wait BUSY bit to 0 */
    while(RESET != ospi_flag_get(ospi_periph, OSPI_FLAG_BUSY)) {
    }

    /* configure OSPI memory mapped mode */
    ospi_functional_mode_config(ospi_periph, OSPI_MEMORY_MAPPED);
}

/*!
    \brief      enable memory mapped mode with wrap
    \param[in]  ospi_periph: OSPIx(x=0,1)
    \param[in]  ospi_struct: OSPI parameter initialization stuct members of the structure
                             and the member values are shown as below:
                  prescaler: between 0 and 255
                  fifo_threshold: OSPI_FIFO_THRESHOLD_x (x = 1, 2, ..., 31, 32)
                  sample_shift: OSPI_SAMPLE_SHIFTING_NONE, OSPI_SAMPLE_SHIFTING_HALF_CYCLE
                  device_size: OSPI_MESZ_x_BYTES (x = 2, 4, 8, ..., 512, 1024)
                             OSPI_MESZ_x_KBS (x = 2, 4, 8, ..., 512, 1024)
                             OSPI_MESZ_x_MBS (x = 2, 4, 8, ..., 2048, 4096)
                  cs_hightime: OSPI_CS_HIGH_TIME_x_CYCLE (x = 1, 2, ..., 63, 64)
                  memory_type: OSPI_MICRON_MODE, OSPI_MACRONIX_MODE, OSPI_STANDARD_MODE
                             OSPI_MACRONIX_RAM_MODE,
                  wrap_size: OSPI_DIRECT, OSPI_WRAP_16BYTES, OSPI_WRAP_32BYTES
                           OSPI_WRAP_64BYTES, OSPI_WRAP_128BYTES
                  delay_hold_cycle: OSPI_DELAY_HOLD_NONE, OSPI_DELAY_HOLD_QUARTER_CYCLE
    \param[in]  mode: flash interface mode
                only one parameter can be selected which is shown as below:
      \arg        SPI_MODE: SPI mode
      \arg        OSPI_MODE: OSPI mode
    \param[in]  addr_size: the size of address
    \retval     none
*/
void ospi_flash_memory_map_mode_wrap_enable(uint32_t ospi_periph, ospi_parameter_struct *ospi_struct, interface_mode mode, addr_size addr_size)
{
    ospi_regular_cmd_struct cmd_struct = {0};

    /* initialize read command */
    if(SPI_MODE == mode) {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_1_LINE;
    } else {
        cmd_struct.ins_mode = OSPI_INSTRUCTION_8_LINES;
    }
    if(GD25X512ME_3BYTES_SIZE == addr_size) {
        cmd_struct.instruction = GD25X512ME_OCTAL_IO_FAST_READ_CMD;
        cmd_struct.addr_size = OSPI_ADDRESS_24_BITS;
    } else {
        cmd_struct.instruction = GD25X512ME_4_BYTE_ADDR_OCTAL_IO_FAST_READ_CMD;
        cmd_struct.addr_size = OSPI_ADDRESS_32_BITS;
    }
    cmd_struct.operation_type = OSPI_OPTYPE_WRAP_CFG;
    cmd_struct.ins_size = OSPI_INSTRUCTION_8_BITS;
    cmd_struct.addr_mode = OSPI_ADDRESS_8_LINES;
    cmd_struct.addr_dtr_mode = OSPI_ADDRDTR_MODE_DISABLE;
    cmd_struct.alter_bytes_mode = OSPI_ALTERNATE_BYTES_NONE;
    cmd_struct.alter_bytes_size = OSPI_ALTERNATE_BYTES_24_BITS;
    cmd_struct.alter_bytes_dtr_mode = OSPI_ABDTR_MODE_DISABLE;
    cmd_struct.data_mode = OSPI_DATA_8_LINES;
    cmd_struct.data_dtr_mode = OSPI_DADTR_MODE_DISABLE;
    cmd_struct.dummy_cycles = OSPI_DUMYC_CYCLES_16;

    /* send the command */
    ospi_command_config(ospi_periph, ospi_struct, &cmd_struct);

    /* initialize program command */
    cmd_struct.operation_type = OSPI_OPTYPE_WRITE_CFG;
    cmd_struct.dummy_cycles = OSPI_DUMYC_CYCLES_0;

    if(GD25X512ME_3BYTES_SIZE == addr_size) {
        cmd_struct.instruction = GD25X512ME_EXT_OCTAL_PAGE_PROG_CMD;
    } else {
        cmd_struct.instruction = GD25X512ME_4_BYTE_EXT_OCTAL_PAGE_PROG_CMD;
    }

    /* send the command */
    ospi_command_config(ospi_periph, ospi_struct, &cmd_struct);

    /* wait BUSY bit to 0 */
    while(RESET != ospi_flag_get(ospi_periph, OSPI_FLAG_BUSY)) {
    }

    /* configure OSPI memory mapped mode */
    ospi_functional_mode_config(ospi_periph, OSPI_MEMORY_MAPPED);
}
