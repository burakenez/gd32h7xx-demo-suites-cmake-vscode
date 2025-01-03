/*!
    \file    sdcard.c
    \brief   SD card driver 
    
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

#include "sdcard.h"
#include "gd32h7xx_sdio.h"
#include <stddef.h>

/* card status of R1 definitions */
#define SD_R1_OUT_OF_RANGE                  BIT(31)                   /* command's argument was out of the allowed range */
#define SD_R1_ADDRESS_ERROR                 BIT(30)                   /* misaligned address which did not match the block length */
#define SD_R1_BLOCK_LEN_ERROR               BIT(29)                   /* transferred block length is not allowed */
#define SD_R1_ERASE_SEQ_ERROR               BIT(28)                   /* an error in the sequence of erase commands occurred */
#define SD_R1_ERASE_PARAM                   BIT(27)                   /* an invalid selection of write-blocks for erase occurred */
#define SD_R1_WP_VIOLATION                  BIT(26)                   /* the host attempts to write to a protected block or to the temporary or permanent write protected card */
#define SD_R1_CARD_IS_LOCKED                BIT(25)                   /* the card is locked by the host */
#define SD_R1_LOCK_UNLOCK_FAILED            BIT(24)                   /* a sequence or password error has been detected in lock/unlock card command */
#define SD_R1_COM_CRC_ERROR                 BIT(23)                   /* CRC check of the previous command failed */
#define SD_R1_ILLEGAL_COMMAND               BIT(22)                   /* command not legal for the card state */
#define SD_R1_CARD_ECC_FAILED               BIT(21)                   /* card internal ECC was applied but failed to correct the data */
#define SD_R1_CC_ERROR                      BIT(20)                   /* internal card controller error */
#define SD_R1_GENERAL_UNKNOWN_ERROR         BIT(19)                   /* a general or an unknown error occurred during the operation */
#define SD_R1_CSD_OVERWRITE                 BIT(16)                   /* read only section of the CSD does not match or attempt to reverse the copy or permanent WP bits */
#define SD_R1_WP_ERASE_SKIP                 BIT(15)                   /* partial address space was erased */
#define SD_R1_CARD_ECC_DISABLED             BIT(14)                   /* command has been executed without using the internal ECC */
#define SD_R1_ERASE_RESET                   BIT(13)                   /* an erase sequence was cleared before executing */
#define SD_R1_READY_FOR_DATA                BIT(8)                    /* correspond to buffer empty signaling on the bus */
#define SD_R1_APP_CMD                       BIT(5)                    /* card will expect ACMD */
#define SD_R1_AKE_SEQ_ERROR                 BIT(3)                    /* error in the sequence of the authentication process */
#define SD_R1_ERROR_BITS                    ((uint32_t)0xFDF9E008U)   /* all the R1 error bits */

/* card status of R6 definitions */
#define SD_R6_COM_CRC_ERROR                 BIT(15)                   /* CRC check of the previous command failed */
#define SD_R6_ILLEGAL_COMMAND               BIT(14)                   /* command not legal for the card state */
#define SD_R6_GENERAL_UNKNOWN_ERROR         BIT(13)                   /* a general or an unknown error occurred during the operation */

/* card state */
#define SD_CARDSTATE_IDLE                   ((uint8_t)0x00)           /* card is in idle state */
#define SD_CARDSTATE_READY                  ((uint8_t)0x01)           /* card is in ready state */
#define SD_CARDSTATE_IDENTIFICAT            ((uint8_t)0x02)           /* card is in identificat state */
#define SD_CARDSTATE_STANDBY                ((uint8_t)0x03)           /* card is in standby state */
#define SD_CARDSTATE_TRANSFER               ((uint8_t)0x04)           /* card is in transfer state */
#define SD_CARDSTATE_DATA                   ((uint8_t)0x05)           /* card is in data sending state */
#define SD_CARDSTATE_RECEIVING              ((uint8_t)0x06)           /* card is in receiving state */
#define SD_CARDSTATE_PROGRAMMING            ((uint8_t)0x07)           /* card is in programming state */
#define SD_CARDSTATE_DISCONNECT             ((uint8_t)0x08)           /* card is in disconnect state */
#define SD_CARDSTATE_LOCKED                 ((uint32_t)0x02000000U)   /* card is in locked state */

#define SD_CHECK_PATTERN                    ((uint32_t)0x000001AAU)    /* check pattern for CMD8 */
#define SD_VOLTAGE_WINDOW                   ((uint32_t)0x80100000U)    /* host 3.3V request in ACMD41 */
#define SD_VOLTAGE_18V                      ((uint32_t)0x01000000U)    /* host 1.8V request in ACMD41 */

/* parameters for ACMD41(voltage validation) */
#define SD_HIGH_CAPACITY                    ((uint32_t)0x40000000U)    /* high capacity SD memory card */
#define SD_STD_CAPACITY                     ((uint32_t)0x00000000U)    /* standard capacity SD memory card */

/* SD bus width, check SCR register */
#define SD_BUS_WIDTH_4BIT                   ((uint32_t)0x00040000U)    /* 4-bit width bus mode */
#define SD_BUS_WIDTH_1BIT                   ((uint32_t)0x00010000U)    /* 1-bit width bus mode */

/* masks for SCR register */
#define SD_MASK_0_7BITS                     ((uint32_t)0x000000FFU)    /* mask [7:0] bits */
#define SD_MASK_8_15BITS                    ((uint32_t)0x0000FF00U)    /* mask [15:8] bits */
#define SD_MASK_16_23BITS                   ((uint32_t)0x00FF0000U)    /* mask [23:16] bits */
#define SD_MASK_24_31BITS                   ((uint32_t)0xFF000000U)    /* mask [31:24] bits */

#define SDIO_FIFO_ADDR                      SDIO_FIFO(SDIO)            /* address of SDIO_FIFO */
#define SD_FIFOHALF_WORDS                   ((uint32_t)0x00000008U)    /* words of FIFO half full/empty */
#define SD_FIFOHALF_BYTES                   ((uint32_t)0x00000020U)    /* bytes of FIFO half full/empty */

#define SD_DATATIMEOUT                      ((uint32_t)0xFFFF0000U)    /* DSM data timeout */
#define SD_MAX_VOLT_VALIDATION              ((uint32_t)0x0000FFFFU)    /* the maximum times of voltage validation */
#define SD_MAX_DATA_LENGTH                  ((uint32_t)0x01FFFFFFU)    /* the maximum length of data */
#define SD_ALLZERO                          ((uint32_t)0x00000000U)    /* all zero */
#define SD_RCA_SHIFT                        ((uint8_t)0x10U)           /* RCA shift bits */

/* user can according to need to change the macro values */
#define SD_CLK_DIV_INIT                     ((uint32_t)0x01F4)        /* SD clock division in initilization phase */
#define SD_CLK_DIV_TRANS_DSPEED             ((uint32_t)0x0008)        /* SD clock division in default speed transmission phase */
#define SD_CLK_DIV_TRANS_HSPEED             ((uint32_t)0x0004)        /* SD clock division in high speed transmission phase */
#define SD_CLK_DIV_TRANS_SDR25SPEED         ((uint32_t)0x0004)        /* SD clock division in SDR25 high speed transmission phase */
#define SD_CLK_DIV_TRANS_SDR50SPEED         ((uint32_t)0x0002)        /* SD clock division in SDR50 high speed transmission phase */
#define SD_CLK_DIV_TRANS_SDR104SPEED        ((uint32_t)0x0001)        /* SD clock division in SDR104 high speed transmission phase */
#define SD_CLK_DIV_TRANS_DDR50SPEED         ((uint32_t)0x0004)        /* SD clock division in DDR50 high speed transmission phase */

#define SDIO_MASK_INTC_FLAGS                ((uint32_t)0x1FE00FFF)    /* mask flags of SDIO_INTC */
#define SDIO_MASK_CMD_FLAGS                 ((uint32_t)0x002000C5)    /* mask flags of CMD FLAGS */
#define SDIO_MASK_DATA_FLAGS                ((uint32_t)0x18000F3A)    /* mask flags of DATA FLAGS */

uint32_t sd_scr[2] = {0, 0};                                          /* content of SCR register */

static sdio_card_type_enum cardtype = SDIO_STD_CAPACITY_SD_CARD_V1_1; /* SD card type */
static uint32_t sdcardtype = SD_STD_CAPACITY;                         /* SD card capacity level, for ACMD41 parameter */
uint32_t cardcapacity = SD_SDSC;                                      /* SD card capacity type */

static uint32_t sd_csd[4] = {0, 0, 0, 0};                             /* content of CSD register */
static uint32_t sd_cid[4] = {0, 0, 0, 0};                             /* content of CID register */
static uint16_t sd_rca = 0U;                                          /* RCA of SD card */
static uint32_t transmode = SD_POLLING_MODE;
static uint32_t totalnumber_bytes = 0U, stopcondition = 0U;

static __IO sd_error_enum transerror = SD_OK;
static __IO uint32_t transend = 0U, number_bytes = 0U;

/* check if the command sent error occurs */
static sd_error_enum cmdsent_error_check(void);
/* check if error occurs for R1 response */
static sd_error_enum r1_error_check(uint8_t cmdindex);
/* check if error type for R1 response */
static sd_error_enum r1_error_type_check(uint32_t resp);
/* check if error occurs for R2 response */
static sd_error_enum r2_error_check(void);
/* check if error occurs for R3 response */
static sd_error_enum r3_error_check(void);
/* check if error occurs for R6 response */
static sd_error_enum r6_error_check(uint8_t cmdindex, uint16_t *prca);
/* check if error occurs for R7 response */
static sd_error_enum r7_error_check(void);

/* get the state which the card is in */
static sd_error_enum sd_card_state_get(uint8_t *pcardstate);
/* configure the bus width mode */
static sd_error_enum sd_bus_width_config(uint32_t buswidth);
/* get the SCR of corresponding card */
static sd_error_enum sd_scr_get(uint16_t rca, uint32_t *pscr);
/* get the data block size */
static uint32_t sd_datablocksize_get(uint16_t bytesnumber);

/* configure the GPIO of SDIO interface */
static void gpio_config(void);
/* configure the RCU of SDIO */
static void rcu_config(void);
/* configure the DMA for SDIO request */
static void dma_config(uint32_t *srcbuf, uint32_t bufsize);

/*!
    \brief      initialize the SD card and make it in standby state
    \param[in]  none
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_init(void)
{
    sd_error_enum status = SD_OK;
    /* configure the RCU and GPIO, deinitialize the SDIO */
    rcu_config();
    gpio_config();
    sdio_deinit(SDIO);

    /* configure the clock and work voltage */
    status = sd_power_on();
    if(SD_OK != status) {
        return status;
    }

    /* initialize the card and get CID and CSD of the card */
    status = sd_card_init();
    if(SD_OK != status) {
        return status;
    }

    /* configure the SDIO peripheral */
    sdio_clock_config(SDIO, SDIO_SDIOCLKEDGE_RISING, SDIO_CLOCKPWRSAVE_DISABLE, SD_CLK_DIV_INIT);
    sdio_bus_mode_set(SDIO, SDIO_BUSMODE_1BIT);
    sdio_hardware_clock_disable(SDIO);

    return status;
}

/*!
    \brief      initialize the card and get CID and CSD of the card
    \param[in]  none
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_card_init(void)
{
    sd_error_enum status = SD_OK;
    uint16_t temp_rca = 0x01U;

    if(SDIO_POWER_OFF == sdio_power_state_get(SDIO)) {
        status = SD_OPERATION_IMPROPER;
        return status;
    }

    /* the card is not I/O only card */
    if(SDIO_SECURE_DIGITAL_IO_CARD != cardtype) {
        /* send CMD2(SD_CMD_ALL_SEND_CID) to get the CID numbers */
        sdio_command_response_config(SDIO, SD_CMD_ALL_SEND_CID, (uint32_t)0x0, SDIO_RESPONSETYPE_LONG);
        sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
        sdio_csm_enable(SDIO);
        /* check if some error occurs */
        status = r2_error_check();
        if(SD_OK != status) {
            return status;
        }

        /* store the CID numbers */
        sd_cid[0] = sdio_response_get(SDIO, SDIO_RESPONSE0);
        sd_cid[1] = sdio_response_get(SDIO, SDIO_RESPONSE1);
        sd_cid[2] = sdio_response_get(SDIO, SDIO_RESPONSE2);
        sd_cid[3] = sdio_response_get(SDIO, SDIO_RESPONSE3);
    }

    /* the card is SD memory card or the I/O card has the memory portion */
    if((SDIO_STD_CAPACITY_SD_CARD_V1_1 == cardtype) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == cardtype) || (SDIO_STD_CAPACITY_SD_CARD_V3_0 == cardtype) ||
            (SDIO_HIGH_CAPACITY_SD_CARD == cardtype) || (SDIO_SECURE_DIGITAL_IO_COMBO_CARD == cardtype)) {
        /* send CMD3(SEND_RELATIVE_ADDR) to ask the card to publish a new relative address (RCA) */
        sdio_command_response_config(SDIO, SD_CMD_SEND_RELATIVE_ADDR, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
        sdio_csm_enable(SDIO);
        /* check if some error occurs */
        status = r6_error_check(SD_CMD_SEND_RELATIVE_ADDR, &temp_rca);
        if(SD_OK != status) {
            return status;
        }
    }

    if(SDIO_SECURE_DIGITAL_IO_CARD != cardtype) {
        /* the card is not I/O only card */
        sd_rca = temp_rca;

        /* send CMD9(SEND_CSD) to get the addressed card's card-specific data (CSD) */
        sdio_command_response_config(SDIO, (uint32_t)SD_CMD_SEND_CSD, ((uint32_t)temp_rca) << SD_RCA_SHIFT, SDIO_RESPONSETYPE_LONG);
        sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
        sdio_csm_enable(SDIO);
        /* check if some error occurs */
        status = r2_error_check();
        if(SD_OK != status) {
            return status;
        }

        /* store the card-specific data (CSD) */
        sd_csd[0] = sdio_response_get(SDIO, SDIO_RESPONSE0);
        sd_csd[1] = sdio_response_get(SDIO, SDIO_RESPONSE1);
        sd_csd[2] = sdio_response_get(SDIO, SDIO_RESPONSE2);
        sd_csd[3] = sdio_response_get(SDIO, SDIO_RESPONSE3);
    }
    return status;
}

/*!
    \brief      configure the clock and the work voltage, and get the card type
    \param[in]  none
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_power_on(void)
{
    sd_error_enum status = SD_OK;
    uint32_t response = 0U, count = 0U;
    uint8_t busyflag = 0U;
    uint32_t timedelay = 0U;

    /* configure the SDIO peripheral */
    sdio_clock_config(SDIO, SDIO_SDIOCLKEDGE_RISING, SDIO_CLOCKPWRSAVE_DISABLE, SD_CLK_DIV_INIT);
    sdio_bus_mode_set(SDIO, SDIO_BUSMODE_1BIT);
    sdio_hardware_clock_disable(SDIO);
    sdio_power_state_set(SDIO, SDIO_POWER_ON);

    /* time delay for power up*/
    timedelay = 500U;
    while(timedelay > 0U) {
        timedelay--;
    }

    /* send CMD0(GO_IDLE_STATE) to reset the card */
    sdio_command_response_config(SDIO, SD_CMD_GO_IDLE_STATE, (uint32_t)0x0, SDIO_RESPONSETYPE_NO);
    sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
    /* enable the CSM */
    sdio_csm_enable(SDIO);

    /* check if command sent error occurs */
    status = cmdsent_error_check();
    if(SD_OK != status) {
        return status;
    }

    /* send CMD8(SEND_IF_COND) to get SD memory card interface condition */
    sdio_command_response_config(SDIO, SD_CMD_SEND_IF_COND, SD_CHECK_PATTERN, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
    sdio_csm_enable(SDIO);

    if(SD_OK == r7_error_check()) {
        /* SD Card 2.0 */
        cardtype = SDIO_STD_CAPACITY_SD_CARD_V2_0;
        sdcardtype = SD_HIGH_CAPACITY;
    }

    /* send CMD55(APP_CMD) to indicate next command is application specific command */
    sdio_command_response_config(SDIO, SD_CMD_APP_CMD, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
    sdio_csm_enable(SDIO);

    if(SD_OK == r1_error_check(SD_CMD_APP_CMD)) {
        /* SD memory card */
        while((!busyflag) && (count < SD_MAX_VOLT_VALIDATION)) {
            /* send CMD55(APP_CMD) to indicate next command is application specific command */
            sdio_command_response_config(SDIO, SD_CMD_APP_CMD, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
            sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
            sdio_csm_enable(SDIO);
            /* check if some error occurs */
            status = r1_error_check(SD_CMD_APP_CMD);
            if(SD_OK != status) {
                return status;
            }

            /* send ACMD41(SD_SEND_OP_COND) to get host capacity support information (HCS) and OCR content */
            sdio_command_response_config(SDIO, SD_APPCMD_SD_SEND_OP_COND, (SD_VOLTAGE_WINDOW | SD_VOLTAGE_18V | sdcardtype), SDIO_RESPONSETYPE_SHORT);
            sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
            sdio_csm_enable(SDIO);
            /* check if some error occurs */
            status = r3_error_check();
            if(SD_OK != status) {
                return status;
            }
            /* get the response and check card power up status bit(busy) */
            response = sdio_response_get(SDIO, SDIO_RESPONSE0);
            busyflag = (uint8_t)((response >> 31U) & (uint32_t)0x01U);
            ++count;
        }
        if(count >= SD_MAX_VOLT_VALIDATION) {
            status = SD_VOLTRANGE_INVALID;
            return status;
        }
        if(response & SD_HIGH_CAPACITY) {
            /* SDHC card */
            cardcapacity = SD_SDHC_SDXC;
            cardtype = SDIO_HIGH_CAPACITY_SD_CARD;
        }
    }
    return status;
}

/*!
    \brief      close the power of SDIO
    \param[in]  none
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_power_off(void)
{
    sd_error_enum status = SD_OK;
    sdio_power_state_set(SDIO, SDIO_POWER_OFF);
    return status;
}

/*!
    \brief      configure the bus mode
    \param[in]  busmode: the bus mode
      \arg        SDIO_BUSMODE_1BIT: 1-bit SDIO card bus mode
      \arg        SDIO_BUSMODE_4BIT: 4-bit SDIO card bus mode
      \arg        SDIO_BUSMODE_8BIT: 8-bit SDIO card bus mode (MMC only)
    \param[in]  speed: the bus speed mode
      \arg        SD_SPEED_DEFAULT: default bus speed
      \arg        SD_SPEED_HIGH: high bus speed
      \arg        SD_SPEED_SDR25: SDR25 bus speed
      \arg        SD_SPEED_SDR50: SDR50 bus speed
      \arg        SD_SPEED_SDR104: SDR104 bus speed
      \arg        SD_SPEED_DDR50: DDR50 bus speed
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_bus_mode_config(uint32_t busmode, uint32_t speed)
{
    sd_error_enum status = SD_OK;
    uint32_t count, clk_div;

    if(SDIO_MULTIMEDIA_CARD == cardtype) {
        /* MMC card doesn't support this function */
        status = SD_FUNCTION_UNSUPPORTED;
        return status;
    } else if((SDIO_STD_CAPACITY_SD_CARD_V1_1 == cardtype) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == cardtype) ||
              (SDIO_HIGH_CAPACITY_SD_CARD == cardtype)) {
        if(SDIO_BUSMODE_8BIT == busmode) {
            /* 8 bit bus mode doesn't support */
            status = SD_FUNCTION_UNSUPPORTED;
            return status;
        } else if(SDIO_BUSMODE_4BIT == busmode) {
            /* configure SD bus width and the SDIO */
            status = sd_bus_width_config(SD_BUS_WIDTH_4BIT);
            if(SD_OK == status) {
                sdio_bus_mode_set(SDIO, busmode);
            }
        } else if(SDIO_BUSMODE_1BIT == busmode) {
            /* configure SD bus width and the SDIO */
            status = sd_bus_width_config(SD_BUS_WIDTH_1BIT);
            if(SD_OK == status) {
                sdio_bus_mode_set(SDIO, busmode);
            }
        } else {
            status = SD_PARAMETER_INVALID;
        }
    }

    if((speed != SD_SPEED_DEFAULT) && (speed != SD_SPEED_HIGH)) {
        /* switch UHS-I speed mode */
        switch(speed) {
        case SD_SPEED_SDR25:
            clk_div = SD_CLK_DIV_TRANS_SDR25SPEED;
            break;
        case SD_SPEED_SDR50:
            clk_div = SD_CLK_DIV_TRANS_SDR50SPEED;
            break;
        case SD_SPEED_SDR104:
            clk_div = SD_CLK_DIV_TRANS_SDR104SPEED;
            break;
        case SD_SPEED_DDR50:
            clk_div = SD_CLK_DIV_TRANS_DDR50SPEED;
            break;
        default:
            clk_div = SD_CLK_DIV_TRANS_DSPEED;
            break;
        }

        /* send CMD16(SET_BLOCKLEN) to set the block length */
        sdio_command_response_config(SDIO, SD_CMD_SET_BLOCKLEN, (uint32_t)64, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
        sdio_csm_enable(SDIO);

        /* check if some error occurs */
        status = r1_error_check(SD_CMD_SET_BLOCKLEN);
        if(SD_OK != status) {
            return status;
        }

        sdio_data_config(SDIO, SD_DATATIMEOUT, 64, SDIO_DATABLOCKSIZE_64BYTES);
        sdio_data_transfer_config(SDIO, SDIO_TRANSDIRECTION_TOSDIO, SDIO_TRANSMODE_BLOCKCOUNT);
        sdio_dsm_enable(SDIO);

        /* SDR25 0x80FFFF01U  SDR50 0x80FF1F02U  SDR104 0x80FF1F03U  DDR50 0x80FF1F04U */
        sdio_command_response_config(SDIO, SD_APPCMD_SET_BUS_WIDTH, speed, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
        sdio_csm_enable(SDIO);
        /* check if some error occurs */
        status = r1_error_check(SD_APPCMD_SET_BUS_WIDTH);
        if(SD_OK != status) {
            return status;
        }

        while(!sdio_flag_get(SDIO, SDIO_FLAG_DTCRCERR | SDIO_FLAG_DTTMOUT | SDIO_FLAG_RXORE | SDIO_FLAG_DTBLKEND | SDIO_FLAG_DTEND)) {
            if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_RFH)) {
                /* at least 8 words can be read in the FIFO */
                for(count = 0; count < SD_FIFOHALF_WORDS; count++) {
                    sdio_data_read(SDIO);
                }
            }
        }
        /* clear the SDIO_INTC flags */
        sdio_flag_clear(SDIO, SDIO_MASK_INTC_FLAGS);
        /* change the clock to UHS-I speed , user according to the speed configuration */
        sdio_clock_config(SDIO, SDIO_SDIOCLKEDGE_RISING, SDIO_CLOCKPWRSAVE_DISABLE, clk_div);
        if(speed == SD_SPEED_DDR50) {
            /* set SDIO data rate */
            sdio_data_rate_set(SDIO, SDIO_DATA_RATE_DDR);
        }
        sdio_hardware_clock_enable(SDIO);
    } else if(speed == SD_SPEED_HIGH) {
        /* change the clock to high speed , user according to the speed configuration */
        sdio_clock_config(SDIO, SDIO_SDIOCLKEDGE_RISING, SDIO_CLOCKPWRSAVE_DISABLE, SD_CLK_DIV_TRANS_HSPEED);
        sdio_hardware_clock_enable(SDIO);
    } else if(speed == SD_SPEED_DEFAULT) {
        /* change the clock to default speed , user according to the speed configuration */
        sdio_clock_config(SDIO, SDIO_SDIOCLKEDGE_RISING, SDIO_CLOCKPWRSAVE_DISABLE, SD_CLK_DIV_TRANS_DSPEED);
        sdio_hardware_clock_enable(SDIO);
    }
    return status;
}

/*!
    \brief      switch 1.8V power level of SD card
    \param[in]  none
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_card_voltage_switch(void)
{
    sd_error_enum status = SD_OK;
    /* send CMD11(SD_CMD_VOLATAGE_SWITCH) switch to 1.8V bus signaling level */
    sdio_command_response_config(SDIO, SD_CMD_VOLATAGE_SWITCH, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
    sdio_csm_enable(SDIO);

    status = r1_error_check(SD_CMD_VOLATAGE_SWITCH);
    return status;
}

/*!
    \brief      configure the mode of transmission
    \param[in]  txmode: transfer mode
      \arg        SD_DMA_MODE: DMA mode
      \arg        SD_POLLING_MODE: polling mode
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_transfer_mode_config(uint32_t txmode)
{
    sd_error_enum status = SD_OK;
    /* set the transfer mode */
    if((SD_DMA_MODE == txmode) || (SD_POLLING_MODE == txmode)) {
        transmode = txmode;
    } else {
        status = SD_PARAMETER_INVALID;
    }
    return status;
}

/*!
    \brief      read a block data into a buffer from the specified address of a card
    \param[out] preadbuffer: a pointer that store a block read data
    \param[in]  readaddr: the read data address
    \param[in]  blocksize: the data block size
    \retval     sd_error_enum
*/
sd_error_enum sd_block_read(uint32_t *preadbuffer, uint32_t readaddr, uint16_t blocksize)
{
    /* initialize the variables */
    sd_error_enum status = SD_OK;
    uint32_t count = 0U, align = 0U, datablksize = SDIO_DATABLOCKSIZE_1BYTE, *ptempbuff = preadbuffer;
    __IO uint32_t timeout = 0U;

    if(NULL == preadbuffer) {
        status = SD_PARAMETER_INVALID;
        return status;
    }

    transerror = SD_OK;
    transend = 0U;
    totalnumber_bytes = 0U;
    /* clear all DSM configuration */
    sdio_data_config(SDIO, 0U, 0U, SDIO_DATABLOCKSIZE_1BYTE);
    sdio_data_transfer_config(SDIO, SDIO_TRANSMODE_BLOCKCOUNT, SDIO_TRANSDIRECTION_TOCARD);
    sdio_dsm_disable(SDIO);
    sdio_idma_disable(SDIO);

    /* check whether the card is locked */
    if(sdio_response_get(SDIO, SDIO_RESPONSE0) & SD_CARDSTATE_LOCKED) {
        status = SD_LOCK_UNLOCK_FAILED;
        return status;
    }

    /* blocksize is fixed in 512B for SDHC card */
    if(SDIO_HIGH_CAPACITY_SD_CARD != cardtype) {
        readaddr *= 512U;
    } else {
        blocksize = 512U;
    }

    align = blocksize & (blocksize - 1U);

    if((blocksize > 0U) && (blocksize <= 2048U) && (0U == align)) {
        datablksize = sd_datablocksize_get(blocksize);
        /* send CMD16(SET_BLOCKLEN) to set the block length */
        sdio_command_response_config(SDIO, SD_CMD_SET_BLOCKLEN, (uint32_t)blocksize, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
        sdio_csm_enable(SDIO);

        /* check if some error occurs */
        status = r1_error_check(SD_CMD_SET_BLOCKLEN);
        if(SD_OK != status) {
            return status;
        }
    } else {
        status = SD_PARAMETER_INVALID;
        return status;
    }

    stopcondition = 0U;
    totalnumber_bytes = (uint32_t)blocksize;

    if(SD_POLLING_MODE == transmode) {

        /* configure SDIO data transmisson */
        sdio_data_config(SDIO, SD_DATATIMEOUT, totalnumber_bytes, datablksize);
        sdio_data_transfer_config(SDIO, SDIO_TRANSMODE_BLOCKCOUNT, SDIO_TRANSDIRECTION_TOSDIO);
        sdio_trans_start_enable(SDIO);


        /* send CMD17(READ_SINGLE_BLOCK) to read a block */
        sdio_command_response_config(SDIO, SD_CMD_READ_SINGLE_BLOCK, (uint32_t)readaddr, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
        sdio_csm_enable(SDIO);
        /* check if some error occurs */
        status = r1_error_check(SD_CMD_READ_SINGLE_BLOCK);
        if(SD_OK != status) {
            return status;
        }

        /* polling mode */
        while(!sdio_flag_get(SDIO, SDIO_FLAG_DTCRCERR | SDIO_FLAG_DTTMOUT | SDIO_FLAG_RXORE | SDIO_FLAG_DTBLKEND | SDIO_FLAG_DTEND)) {
            if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_RFH)) {
                /* at least 8 words can be read in the FIFO */
                for(count = 0U; count < SD_FIFOHALF_WORDS; count++) {
                    *(ptempbuff + count) = sdio_data_read(SDIO);
                }
                ptempbuff += SD_FIFOHALF_WORDS;
            }
        }

        sdio_trans_start_disable(SDIO);

        /* whether some error occurs and return it */
        if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_DTCRCERR)) {
            status = SD_DATA_CRC_ERROR;
            sdio_flag_clear(SDIO, SDIO_FLAG_DTCRCERR);
            return status;
        } else if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_DTTMOUT)) {
            status = SD_DATA_TIMEOUT;
            sdio_flag_clear(SDIO, SDIO_FLAG_DTTMOUT);
            return status;
        } else if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_RXORE)) {
            status = SD_RX_OVERRUN_ERROR;
            sdio_flag_clear(SDIO, SDIO_FLAG_RXORE);
            return status;
        } else {
            /* if else end */
        }

        while((SET != sdio_flag_get(SDIO, SDIO_FLAG_RFE)) && (SET == sdio_flag_get(SDIO, SDIO_FLAG_DATSTA))) {
            *ptempbuff = sdio_data_read(SDIO);
            ++ptempbuff;
        }
        /* clear the DATA_FLAGS flags */
        sdio_flag_clear(SDIO, SDIO_MASK_DATA_FLAGS);
    } else if(SD_DMA_MODE == transmode) {
        /* DMA mode */
        /* enable the SDIO corresponding interrupts and DMA function */
        sdio_interrupt_enable(SDIO, SDIO_INT_CCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_RXORE | SDIO_INT_DTEND);
        dma_config(preadbuffer, (uint32_t)(blocksize >> 5));
        sdio_idma_enable(SDIO);

        /* configure SDIO data transmisson */
        sdio_data_config(SDIO, SD_DATATIMEOUT, totalnumber_bytes, datablksize);
        sdio_data_transfer_config(SDIO, SDIO_TRANSMODE_BLOCKCOUNT, SDIO_TRANSDIRECTION_TOSDIO);
        sdio_trans_start_enable(SDIO);

        /* send CMD17(READ_SINGLE_BLOCK) to read a block */
        sdio_command_response_config(SDIO, SD_CMD_READ_SINGLE_BLOCK, (uint32_t)readaddr, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
        sdio_csm_enable(SDIO);
        /* check if some error occurs */
        status = r1_error_check(SD_CMD_READ_SINGLE_BLOCK);
        if(SD_OK != status) {
            return status;
        }

        while((0U == transend) && (SD_OK == transerror)) {
        }
        if(SD_OK != transerror) {
            return transerror;
        }
    } else {
        status = SD_PARAMETER_INVALID;
    }
    return status;
}

/*!
    \brief      read multiple blocks data into a buffer from the specified address of a card
    \param[out] preadbuffer: a pointer that store multiple blocks read data
    \param[in]  readaddr: the read data address
    \param[in]  blocksize: the data block size
    \param[in]  blocksnumber: number of blocks that will be read
    \retval     sd_error_enum
*/
sd_error_enum sd_multiblocks_read(uint32_t *preadbuffer, uint32_t readaddr, uint16_t blocksize, uint32_t blocksnumber)
{
    /* initialize the variables */
    sd_error_enum status = SD_OK;
    uint32_t count = 0U, align = 0U, datablksize = SDIO_DATABLOCKSIZE_1BYTE, *ptempbuff = preadbuffer;
    __IO uint32_t timeout = 0U;

    if(NULL == preadbuffer) {
        status = SD_PARAMETER_INVALID;
        return status;
    }

    transerror = SD_OK;
    transend = 0U;
    totalnumber_bytes = 0U;
    /* clear all DSM configuration */
    sdio_data_config(SDIO, 0U, 0U, SDIO_DATABLOCKSIZE_1BYTE);
    sdio_data_transfer_config(SDIO, SDIO_TRANSMODE_BLOCKCOUNT, SDIO_TRANSDIRECTION_TOCARD);
    sdio_dsm_disable(SDIO);
    sdio_idma_disable(SDIO);

    /* check whether the card is locked */
    if(sdio_response_get(SDIO, SDIO_RESPONSE0) & SD_CARDSTATE_LOCKED) {
        status = SD_LOCK_UNLOCK_FAILED;
        return status;
    }

    /* blocksize is fixed in 512B for SDHC card */
    if(SDIO_HIGH_CAPACITY_SD_CARD != cardtype) {
        readaddr *= 512U;
    } else {
        blocksize = 512U;
    }

    align = blocksize & (blocksize - 1U);
    if((blocksize > 0U) && (blocksize <= 2048U) && (0U == align)) {
        datablksize = sd_datablocksize_get(blocksize);
        /* send CMD16(SET_BLOCKLEN) to set the block length */
        sdio_command_response_config(SDIO, SD_CMD_SET_BLOCKLEN, (uint32_t)blocksize, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
        sdio_csm_enable(SDIO);

        /* check if some error occurs */
        status = r1_error_check(SD_CMD_SET_BLOCKLEN);
        if(SD_OK != status) {
            return status;
        }
    } else {
        status = SD_PARAMETER_INVALID;
        return status;
    }

    if(blocksnumber > 1U) {
        if(blocksnumber * blocksize > SD_MAX_DATA_LENGTH) {
            /* exceeds the maximum length */
            status = SD_PARAMETER_INVALID;
            return status;
        }

        stopcondition = 1U;
        totalnumber_bytes = blocksnumber * blocksize;

        if(SD_POLLING_MODE == transmode) {

            /* configure SDIO data transmisson */
            sdio_data_config(SDIO, SD_DATATIMEOUT, totalnumber_bytes, datablksize);
            sdio_data_transfer_config(SDIO, SDIO_TRANSMODE_BLOCKCOUNT, SDIO_TRANSDIRECTION_TOSDIO);
            sdio_trans_start_enable(SDIO);

            /* send CMD18(READ_MULTIPLE_BLOCK) to read multiple blocks */
            sdio_command_response_config(SDIO, SD_CMD_READ_MULTIPLE_BLOCK, readaddr, SDIO_RESPONSETYPE_SHORT);
            sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
            sdio_csm_enable(SDIO);

            /* check if some error occurs */
            status = r1_error_check(SD_CMD_READ_MULTIPLE_BLOCK);
            if(SD_OK != status) {
                return status;
            }

            /* polling mode */
            while(!sdio_flag_get(SDIO, SDIO_FLAG_DTCRCERR | SDIO_FLAG_DTTMOUT | SDIO_FLAG_RXORE | SDIO_FLAG_DTBLKEND | SDIO_FLAG_DTEND)) {
                if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_RFH)) {
                    /* at least 8 words can be read in the FIFO */
                    for(count = 0U; count < SD_FIFOHALF_WORDS; count++) {
                        *(ptempbuff + count) = sdio_data_read(SDIO);
                    }
                    ptempbuff += SD_FIFOHALF_WORDS;
                }
            }

            /* whether some error occurs and return it */
            if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_DTCRCERR)) {
                status = SD_DATA_CRC_ERROR;
                sdio_flag_clear(SDIO, SDIO_FLAG_DTCRCERR);
                return status;
            } else if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_DTTMOUT)) {
                status = SD_DATA_TIMEOUT;
                sdio_flag_clear(SDIO, SDIO_FLAG_DTTMOUT);
                return status;
            } else if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_RXORE)) {
                status = SD_RX_OVERRUN_ERROR;
                sdio_flag_clear(SDIO, SDIO_FLAG_RXORE);
                return status;
            } else {
                /* if else end */
            }

            while((SET != sdio_flag_get(SDIO, SDIO_FLAG_RFE)) && (SET == sdio_flag_get(SDIO, SDIO_FLAG_DATSTA))) {
                *ptempbuff = sdio_data_read(SDIO);
                ++ptempbuff;
            }

            sdio_trans_start_disable(SDIO);

            if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_DTEND)) {
                if((SDIO_STD_CAPACITY_SD_CARD_V1_1 == cardtype) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == cardtype) ||
                        (SDIO_HIGH_CAPACITY_SD_CARD == cardtype)) {
                    /* send CMD12(STOP_TRANSMISSION) to stop transmission */
                    sdio_command_response_config(SDIO, SD_CMD_STOP_TRANSMISSION, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
                    sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
                    sdio_csm_enable(SDIO);
                    /* check if some error occurs */
                    status = r1_error_check(SD_CMD_STOP_TRANSMISSION);
                    if(SD_OK != status) {
                        return status;
                    }
                }
            }
            /* clear the DATA_FLAGS flags */
            sdio_flag_clear(SDIO, SDIO_MASK_DATA_FLAGS);
        } else if(SD_DMA_MODE == transmode) {
            /* DMA mode */
            /* enable the SDIO corresponding interrupts and DMA function */
            sdio_interrupt_enable(SDIO, SDIO_INT_CCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_RXORE | SDIO_INT_DTEND);
            dma_config(preadbuffer, (uint32_t)(blocksize >> 5));
            sdio_idma_enable(SDIO);

            /* configure SDIO data transmisson */
            sdio_data_config(SDIO, SD_DATATIMEOUT, totalnumber_bytes, datablksize);
            sdio_data_transfer_config(SDIO, SDIO_TRANSMODE_BLOCKCOUNT, SDIO_TRANSDIRECTION_TOSDIO);
            sdio_trans_start_enable(SDIO);

            /* send CMD18(READ_MULTIPLE_BLOCK) to read multiple blocks */
            sdio_command_response_config(SDIO, SD_CMD_READ_MULTIPLE_BLOCK, readaddr, SDIO_RESPONSETYPE_SHORT);
            sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
            sdio_csm_enable(SDIO);
            /* check if some error occurs */
            status = r1_error_check(SD_CMD_READ_MULTIPLE_BLOCK);
            if(SD_OK != status) {
                return status;
            }

            while((0U == transend) && (SD_OK == transerror)) {
            }
            if(SD_OK != transerror) {
                return transerror;
            }
        } else {
            status = SD_PARAMETER_INVALID;
        }
    }
    return status;
}

/*!
    \brief      write a block data to the specified address of a card
    \param[in]  pwritebuffer: a pointer that store a block data to be transferred
    \param[in]  writeaddr: the read data address
    \param[in]  blocksize: the data block size
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_block_write(uint32_t *pwritebuffer, uint32_t writeaddr, uint16_t blocksize)
{
    /* initialize the variables */
    sd_error_enum status = SD_OK;
    uint8_t cardstate = 0U;
    uint32_t count = 0U, align = 0U, datablksize = SDIO_DATABLOCKSIZE_1BYTE, *ptempbuff = pwritebuffer;
    uint32_t transbytes = 0U, restwords = 0U, response = 0U;
    __IO uint32_t timeout = 0U;

    if(NULL == pwritebuffer) {
        status = SD_PARAMETER_INVALID;
        return status;
    }

    transerror = SD_OK;
    transend = 0U;
    totalnumber_bytes = 0U;
    /* clear all DSM configuration */
    sdio_data_config(SDIO, 0U, 0U, SDIO_DATABLOCKSIZE_1BYTE);
    sdio_data_transfer_config(SDIO, SDIO_TRANSMODE_BLOCKCOUNT, SDIO_TRANSDIRECTION_TOCARD);
    sdio_dsm_disable(SDIO);
    sdio_idma_disable(SDIO);

    /* check whether the card is locked */
    if(sdio_response_get(SDIO, SDIO_RESPONSE0) & SD_CARDSTATE_LOCKED) {
        status = SD_LOCK_UNLOCK_FAILED;
        return status;
    }

    /* blocksize is fixed in 512B for SDHC card */
    if(SDIO_HIGH_CAPACITY_SD_CARD != cardtype) {
        writeaddr *= 512U;
    } else {
        blocksize = 512U;
    }

    align = blocksize & (blocksize - 1U);
    if((blocksize > 0U) && (blocksize <= 2048U) && (0U == align)) {
        datablksize = sd_datablocksize_get(blocksize);
        /* send CMD16(SET_BLOCKLEN) to set the block length */
        sdio_command_response_config(SDIO, SD_CMD_SET_BLOCKLEN, (uint32_t)blocksize, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
        sdio_csm_enable(SDIO);

        /* check if some error occurs */
        status = r1_error_check(SD_CMD_SET_BLOCKLEN);
        if(SD_OK != status) {
            return status;
        }
    } else {
        status = SD_PARAMETER_INVALID;
        return status;
    }

    /* send CMD13(SEND_STATUS), addressed card sends its status registers */
    sdio_command_response_config(SDIO, SD_CMD_SEND_STATUS, (uint32_t)sd_rca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
    sdio_csm_enable(SDIO);
    /* check if some error occurs */
    status = r1_error_check(SD_CMD_SEND_STATUS);
    if(SD_OK != status) {
        return status;
    }

    response = sdio_response_get(SDIO, SDIO_RESPONSE0);
    timeout = 100000U;

    while((0U == (response & SD_R1_READY_FOR_DATA)) && (timeout > 0U)) {
        /* continue to send CMD13 to polling the state of card until buffer empty or timeout */
        --timeout;
        /* send CMD13(SEND_STATUS), addressed card sends its status registers */
        sdio_command_response_config(SDIO, SD_CMD_SEND_STATUS, (uint32_t)sd_rca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
        sdio_csm_enable(SDIO);
        /* check if some error occurs */
        status = r1_error_check(SD_CMD_SEND_STATUS);
        if(SD_OK != status) {
            return status;
        }
        response = sdio_response_get(SDIO, SDIO_RESPONSE0);
    }
    if(0U == timeout) {
        return SD_ERROR;
    }

    stopcondition = 0U;
    totalnumber_bytes = blocksize;


    /* configure the SDIO data transmisson */
    sdio_data_config(SDIO, SD_DATATIMEOUT, totalnumber_bytes, datablksize);
    sdio_data_transfer_config(SDIO, SDIO_TRANSMODE_BLOCKCOUNT, SDIO_TRANSDIRECTION_TOCARD);
    sdio_trans_start_enable(SDIO);

    if(SD_POLLING_MODE == transmode) {
        /* send CMD24(WRITE_BLOCK) to write a block */
        sdio_command_response_config(SDIO, SD_CMD_WRITE_BLOCK, writeaddr, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
        sdio_csm_enable(SDIO);
        /* check if some error occurs */
        status = r1_error_check(SD_CMD_WRITE_BLOCK);
        if(SD_OK != status) {
            return status;
        }

        /* polling mode */
        while(!sdio_flag_get(SDIO, SDIO_FLAG_DTCRCERR | SDIO_FLAG_DTTMOUT | SDIO_FLAG_TXURE | SDIO_FLAG_DTBLKEND | SDIO_FLAG_DTEND)) {
            if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_TFH)) {
                /* at least 8 words can be written into the FIFO */
                if((totalnumber_bytes - transbytes) < SD_FIFOHALF_BYTES) {
                    restwords = (totalnumber_bytes - transbytes) / 4U + (((totalnumber_bytes - transbytes) % 4U == 0U) ? 0U : 1U);
                    for(count = 0U; count < restwords; count++) {
                        sdio_data_write(SDIO, *ptempbuff);
                        ++ptempbuff;
                        transbytes += 4U;
                    }
                } else {
                    for(count = 0U; count < SD_FIFOHALF_WORDS; count++) {
                        sdio_data_write(SDIO, *(ptempbuff + count));
                    }
                    /* 8 words(32 bytes) has been transferred */
                    ptempbuff += SD_FIFOHALF_WORDS;
                    transbytes += SD_FIFOHALF_BYTES;
                }
            }
        }
        sdio_trans_start_disable(SDIO);
        /* whether some error occurs and return it */
        if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_DTCRCERR)) {
            status = SD_DATA_CRC_ERROR;
            sdio_flag_clear(SDIO, SDIO_FLAG_DTCRCERR);
            return status;
        } else if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_DTTMOUT)) {
            status = SD_DATA_TIMEOUT;
            sdio_flag_clear(SDIO, SDIO_FLAG_DTTMOUT);
            return status;
        } else if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_TXURE)) {
            status = SD_TX_UNDERRUN_ERROR;
            sdio_flag_clear(SDIO, SDIO_FLAG_TXURE);
            return status;
        } else {
            /* if else end */
        }

    } else if(SD_DMA_MODE == transmode) {
        /* DMA mode */
        /* enable the SDIO corresponding interrupts and DMA */
        sdio_interrupt_enable(SDIO, SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_TXURE | SDIO_INT_DTEND);
        dma_config(pwritebuffer, (uint32_t)(blocksize >> 5));
        sdio_idma_enable(SDIO);

        /* send CMD24(WRITE_BLOCK) to write a block */
        sdio_command_response_config(SDIO, SD_CMD_WRITE_BLOCK, writeaddr, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
        sdio_csm_enable(SDIO);
        /* check if some error occurs */
        status = r1_error_check(SD_CMD_WRITE_BLOCK);
        if(SD_OK != status) {
            return status;
        }

        while((0U == transend) && (SD_OK == transerror)) {
        }

        if(SD_OK != transerror) {
            return transerror;
        }
    } else {
        status = SD_PARAMETER_INVALID;
        return status;
    }

    /* clear the DATA_FLAGS flags */
    sdio_flag_clear(SDIO, SDIO_MASK_DATA_FLAGS);
    /* get the card state and wait the card is out of programming and receiving state */
    status = sd_card_state_get(&cardstate);
    while((SD_OK == status) && ((SD_CARDSTATE_PROGRAMMING == cardstate) || (SD_CARDSTATE_RECEIVING == cardstate))) {
        status = sd_card_state_get(&cardstate);
    }
    return status;
}

/*!
    \brief      write multiple blocks data to the specified address of a card
    \param[in]  pwritebuffer: a pointer that store multiple blocks data to be transferred
    \param[in]  writeaddr: the read data address
    \param[in]  blocksize: the data block size
    \param[in]  blocksnumber: number of blocks that will be written
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_multiblocks_write(uint32_t *pwritebuffer, uint32_t writeaddr, uint16_t blocksize, uint32_t blocksnumber)
{
    /* initialize the variables */
    sd_error_enum status = SD_OK;
    uint8_t cardstate = 0U;
    uint32_t count = 0U, align = 0U, datablksize = SDIO_DATABLOCKSIZE_1BYTE, *ptempbuff = pwritebuffer;
    uint32_t transbytes = 0U, restwords = 0U, response = 0U;
    __IO uint32_t timeout = 0U;

    if(NULL == pwritebuffer) {
        status = SD_PARAMETER_INVALID;
        return status;
    }

    transerror = SD_OK;
    transend = 0U;
    totalnumber_bytes = 0U;
    /* clear all DSM configuration */
    sdio_data_config(SDIO, 0U, 0U, SDIO_DATABLOCKSIZE_1BYTE);
    sdio_data_transfer_config(SDIO, SDIO_TRANSMODE_BLOCKCOUNT, SDIO_TRANSDIRECTION_TOCARD);
    sdio_dsm_disable(SDIO);
    sdio_idma_disable(SDIO);

    /* check whether the card is locked */
    if(sdio_response_get(SDIO, SDIO_RESPONSE0) & SD_CARDSTATE_LOCKED) {
        status = SD_LOCK_UNLOCK_FAILED;
        return status;
    }

    /* blocksize is fixed in 512B for SDHC card */
    if(SDIO_HIGH_CAPACITY_SD_CARD != cardtype) {
        writeaddr *= 512U;
    } else {
        blocksize = 512U;
    }

    align = blocksize & (blocksize - 1U);
    if((blocksize > 0U) && (blocksize <= 2048U) && (0U == align)) {
        datablksize = sd_datablocksize_get(blocksize);
        /* send CMD16(SET_BLOCKLEN) to set the block length */
        sdio_command_response_config(SDIO, SD_CMD_SET_BLOCKLEN, (uint32_t)blocksize, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
        sdio_csm_enable(SDIO);

        /* check if some error occurs */
        status = r1_error_check(SD_CMD_SET_BLOCKLEN);
        if(SD_OK != status) {
            return status;
        }
    } else {
        status = SD_PARAMETER_INVALID;
        return status;
    }

    /* send CMD13(SEND_STATUS), addressed card sends its status registers */
    sdio_command_response_config(SDIO, SD_CMD_SEND_STATUS, (uint32_t)sd_rca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
    sdio_csm_enable(SDIO);
    /* check if some error occurs */
    status = r1_error_check(SD_CMD_SEND_STATUS);
    if(SD_OK != status) {
        return status;
    }

    response = sdio_response_get(SDIO, SDIO_RESPONSE0);
    timeout = 100000U;

    while((0U == (response & SD_R1_READY_FOR_DATA)) && (timeout > 0U)) {
        /* continue to send CMD13 to polling the state of card until buffer empty or timeout */
        --timeout;
        /* send CMD13(SEND_STATUS), addressed card sends its status registers */
        sdio_command_response_config(SDIO, SD_CMD_SEND_STATUS, (uint32_t)sd_rca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
        sdio_csm_enable(SDIO);
        /* check if some error occurs */
        status = r1_error_check(SD_CMD_SEND_STATUS);
        if(SD_OK != status) {
            return status;
        }
        response = sdio_response_get(SDIO, SDIO_RESPONSE0);
    }
    if(0U == timeout) {
        return SD_ERROR;
    }

    if(blocksnumber > 1U) {
        if(blocksnumber * blocksize > SD_MAX_DATA_LENGTH) {
            status = SD_PARAMETER_INVALID;
            return status;
        }

        if((SDIO_STD_CAPACITY_SD_CARD_V1_1 == cardtype) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == cardtype) ||
                (SDIO_HIGH_CAPACITY_SD_CARD == cardtype)) {
            /* send CMD55(APP_CMD) to indicate next command is application specific command */
            sdio_command_response_config(SDIO, SD_CMD_APP_CMD, (uint32_t)sd_rca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
            sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
            sdio_csm_enable(SDIO);
            /* check if some error occurs */
            status = r1_error_check(SD_CMD_APP_CMD);
            if(SD_OK != status) {
                return status;
            }

            /* send ACMD23(SET_WR_BLK_ERASE_COUNT) to set the number of write blocks to be preerased before writing */
            sdio_command_response_config(SDIO, SD_APPCMD_SET_WR_BLK_ERASE_COUNT, blocksnumber, SDIO_RESPONSETYPE_SHORT);
            sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
            sdio_csm_enable(SDIO);
            /* check if some error occurs */
            status = r1_error_check(SD_APPCMD_SET_WR_BLK_ERASE_COUNT);
            if(SD_OK != status) {
                return status;
            }
        }

        stopcondition = 1U;
        totalnumber_bytes = blocksnumber * blocksize;

        if(SD_POLLING_MODE == transmode) {
            /* configure the SDIO data transmisson */
            sdio_data_config(SDIO, SD_DATATIMEOUT, totalnumber_bytes, datablksize);
            sdio_data_transfer_config(SDIO, SDIO_TRANSMODE_BLOCKCOUNT, SDIO_TRANSDIRECTION_TOCARD);
            sdio_trans_start_enable(SDIO);

            /* send CMD25(WRITE_MULTIPLE_BLOCK) to continuously write blocks of data */
            sdio_command_response_config(SDIO, SD_CMD_WRITE_MULTIPLE_BLOCK, writeaddr, SDIO_RESPONSETYPE_SHORT);
            sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
            sdio_csm_enable(SDIO);
            /* check if some error occurs */
            status = r1_error_check(SD_CMD_WRITE_MULTIPLE_BLOCK);
            if(SD_OK != status) {
                return status;
            }

            /* polling mode */
            while(!sdio_flag_get(SDIO, SDIO_FLAG_DTCRCERR | SDIO_FLAG_DTTMOUT | SDIO_FLAG_TXURE | SDIO_FLAG_DTBLKEND | SDIO_FLAG_DTEND)) {
                if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_TFH)) {
                    /* at least 8 words can be written into the FIFO */
                    if((totalnumber_bytes - transbytes) < SD_FIFOHALF_BYTES) {
                        restwords = (totalnumber_bytes - transbytes) / 4U + (((totalnumber_bytes - transbytes) % 4U == 0U) ? 0U : 1U);
                        for(count = 0U; count < restwords; count++) {
                            sdio_data_write(SDIO, *ptempbuff);
                            ++ptempbuff;
                            transbytes += 4U;
                        }
                    } else {
                        for(count = 0U; count < SD_FIFOHALF_WORDS; count++) {
                            sdio_data_write(SDIO, *(ptempbuff + count));
                        }
                        /* 8 words(32 bytes) has been transferred */
                        ptempbuff += SD_FIFOHALF_WORDS;
                        transbytes += SD_FIFOHALF_BYTES;
                    }
                }
            }
            sdio_trans_start_disable(SDIO);

            /* whether some error occurs and return it */
            if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_DTCRCERR)) {
                status = SD_DATA_CRC_ERROR;
                sdio_flag_clear(SDIO, SDIO_FLAG_DTCRCERR);
                return status;
            } else if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_DTTMOUT)) {
                status = SD_DATA_TIMEOUT;
                sdio_flag_clear(SDIO, SDIO_FLAG_DTTMOUT);
                return status;
            } else if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_TXURE)) {
                status = SD_TX_UNDERRUN_ERROR;
                sdio_flag_clear(SDIO, SDIO_FLAG_TXURE);
                return status;
            } else {
                /* if else end */
            }

            if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_DTEND)) {
                if((SDIO_STD_CAPACITY_SD_CARD_V1_1 == cardtype) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == cardtype) ||
                        (SDIO_HIGH_CAPACITY_SD_CARD == cardtype)) {
                    /* send CMD12(STOP_TRANSMISSION) to stop transmission */
                    sdio_command_response_config(SDIO, SD_CMD_STOP_TRANSMISSION, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
                    sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
                    sdio_csm_enable(SDIO);
                    /* check if some error occurs */
                    status = r1_error_check(SD_CMD_STOP_TRANSMISSION);
                    if(SD_OK != status) {
                        return status;
                    }
                }
            }
        } else if(SD_DMA_MODE == transmode) {
            /* DMA mode */
            /* enable the SDIO corresponding interrupts and DMA */
            sdio_interrupt_enable(SDIO, SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_TXURE | SDIO_INT_DTEND);
            dma_config(pwritebuffer, (uint32_t)(blocksize >> 5));
            sdio_idma_enable(SDIO);

            /* configure the SDIO data transmisson */
            sdio_data_config(SDIO, SD_DATATIMEOUT, totalnumber_bytes, datablksize);
            sdio_data_transfer_config(SDIO, SDIO_TRANSMODE_BLOCKCOUNT, SDIO_TRANSDIRECTION_TOCARD);
            sdio_trans_start_enable(SDIO);

            /* send CMD25(WRITE_MULTIPLE_BLOCK) to continuously write blocks of data */
            sdio_command_response_config(SDIO, SD_CMD_WRITE_MULTIPLE_BLOCK, writeaddr, SDIO_RESPONSETYPE_SHORT);
            sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
            sdio_csm_enable(SDIO);
            /* check if some error occurs */
            status = r1_error_check(SD_CMD_WRITE_MULTIPLE_BLOCK);
            if(SD_OK != status) {
                return status;
            }
            while((0U == transend) && (SD_OK == transerror)) {
            }

            if(SD_OK != transerror) {
                return transerror;
            }
        } else {
            status = SD_PARAMETER_INVALID;
            return status;
        }
    }
    /* clear the DATA_FLAGS flags */
    sdio_flag_clear(SDIO, SDIO_MASK_DATA_FLAGS);
    /* get the card state and wait the card is out of programming and receiving state */
    status = sd_card_state_get(&cardstate);
    while((SD_OK == status) && ((SD_CARDSTATE_PROGRAMMING == cardstate) || (SD_CARDSTATE_RECEIVING == cardstate))) {
        status = sd_card_state_get(&cardstate);
    }
    return status;
}

/*!
    \brief      erase a continuous area of a card
    \param[in]  startaddr: the start address
    \param[in]  endaddr: the end address
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_erase(uint32_t startaddr, uint32_t endaddr)
{
    /* initialize the variables */
    sd_error_enum status = SD_OK;
    uint32_t count = 0U, clkdiv = 0U;
    __IO uint32_t delay = 0U;
    uint8_t cardstate = 0U, tempbyte = 0U;
    uint16_t tempccc = 0U;

    /* get the card command classes from CSD */
    tempbyte = (uint8_t)((sd_csd[1] & SD_MASK_24_31BITS) >> 24U);
    tempccc = (uint16_t)((uint16_t)tempbyte << 4U);
    tempbyte = (uint8_t)((sd_csd[1] & SD_MASK_16_23BITS) >> 16U);
    tempccc |= ((uint16_t)tempbyte & 0xF0U) >> 4U;
    if(0U == (tempccc & SD_CCC_ERASE)) {
        /* don't support the erase command */
        status = SD_FUNCTION_UNSUPPORTED;
        return status;
    }
    clkdiv = (SDIO_CLKCTL(SDIO) & SDIO_CLKCTL_DIV);
    clkdiv *= 2U;
    delay = 168000U / clkdiv;

    /* check whether the card is locked */
    if(sdio_response_get(SDIO, SDIO_RESPONSE0) & SD_CARDSTATE_LOCKED) {
        status = SD_LOCK_UNLOCK_FAILED;
        return(status);
    }

    /* blocksize is fixed in 512B for SDHC card */
    if(SDIO_HIGH_CAPACITY_SD_CARD != cardtype) {
        startaddr *= 512U;
        endaddr *= 512U;
    }

    if((SDIO_STD_CAPACITY_SD_CARD_V1_1 == cardtype) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == cardtype) ||
            (SDIO_HIGH_CAPACITY_SD_CARD == cardtype)) {
        /* send CMD32(ERASE_WR_BLK_START) to set the address of the first write block to be erased */
        sdio_command_response_config(SDIO, SD_CMD_ERASE_WR_BLK_START, startaddr, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
        sdio_csm_enable(SDIO);
        /* check if some error occurs */
        status = r1_error_check(SD_CMD_ERASE_WR_BLK_START);
        if(SD_OK != status) {
            return status;
        }

        /* send CMD33(ERASE_WR_BLK_END) to set the address of the last write block of the continuous range to be erased */
        sdio_command_response_config(SDIO, SD_CMD_ERASE_WR_BLK_END, endaddr, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
        sdio_csm_enable(SDIO);
        /* check if some error occurs */
        status = r1_error_check(SD_CMD_ERASE_WR_BLK_END);
        if(SD_OK != status) {
            return status;
        }
    }

    /* send CMD38(ERASE) to set the address of the first write block to be erased */
    sdio_command_response_config(SDIO, SD_CMD_ERASE, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
    sdio_csm_enable(SDIO);
    /* check if some error occurs */
    status = r1_error_check(SD_CMD_ERASE);
    if(SD_OK != status) {
        return status;
    }
    /* loop until the counter is reach to the calculated time */
    for(count = 0U; count < delay; count++) {
    }
    /* get the card state and wait the card is out of programming and receiving state */
    status = sd_card_state_get(&cardstate);
    while((SD_OK == status) && ((SD_CARDSTATE_PROGRAMMING == cardstate) || (SD_CARDSTATE_RECEIVING == cardstate))) {
        status = sd_card_state_get(&cardstate);
    }
    return status;
}



/*!
    \brief      process all the interrupts which the corresponding flags are set
    \param[in]  none
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_interrupts_process(void)
{
    transerror = SD_OK;
    if(RESET != sdio_interrupt_flag_get(SDIO, SDIO_INT_FLAG_DTEND)) {
        /* clear DTEND flag */
        sdio_interrupt_flag_clear(SDIO, SDIO_INT_FLAG_DTEND);
        /* disable idma for idma transfer */
        sdio_idma_disable(SDIO);

        /* disable all the interrupts */
        sdio_interrupt_disable(SDIO, SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_DTEND |
                               SDIO_INT_TFH | SDIO_INT_RFH | SDIO_INT_TXURE | SDIO_INT_RXORE);
        sdio_trans_start_disable(SDIO);
        /* send CMD12 to stop data transfer in multipule blocks operation */
        if(1 == stopcondition) {
            transerror = sd_transfer_stop();
        } else {
            transerror = SD_OK;
        }
        transend = 1;
        number_bytes = 0;
        /* clear data flags */
        sdio_flag_clear(SDIO, SDIO_MASK_DATA_FLAGS);
        return transerror;
    }

    if(RESET != sdio_interrupt_flag_get(SDIO, SDIO_INT_FLAG_DTCRCERR | SDIO_INT_FLAG_DTTMOUT | SDIO_INT_FLAG_TXURE | SDIO_INT_FLAG_RXORE)) {
        /* set different errors */
        if(RESET != sdio_interrupt_flag_get(SDIO, SDIO_INT_FLAG_DTCRCERR)) {
            transerror = SD_DATA_CRC_ERROR;
        }
        if(RESET != sdio_interrupt_flag_get(SDIO, SDIO_INT_FLAG_DTTMOUT)) {
            transerror = SD_DATA_TIMEOUT;
        }
        if(RESET != sdio_interrupt_flag_get(SDIO, SDIO_INT_FLAG_TXURE)) {
            transerror = SD_TX_UNDERRUN_ERROR;
        }
        if(RESET != sdio_interrupt_flag_get(SDIO, SDIO_INT_FLAG_RXORE)) {
            transerror = SD_RX_OVERRUN_ERROR;
        }
        /* clear data flags */
        sdio_flag_clear(SDIO, SDIO_MASK_DATA_FLAGS);
        /* disable all the interrupts */
        sdio_interrupt_disable(SDIO, SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_DTEND |
                               SDIO_INT_TXURE | SDIO_INT_RXORE);

        sdio_trans_start_disable(SDIO);
        sdio_fifo_reset_enable(SDIO);
        sdio_fifo_reset_disable(SDIO);
        /* send CMD12 to stop data transfer in multipule blocks operation */
        transerror = sd_transfer_stop();
        sdio_flag_clear(SDIO, SDIO_FLAG_DTABORT);
        number_bytes = 0;

        if(transmode == SD_DMA_MODE) {
            sdio_idma_disable(SDIO);
        }
        return transerror;
    }

    if(RESET != sdio_interrupt_flag_get(SDIO, SDIO_INT_FLAG_IDMAERR)) {
        sdio_interrupt_flag_clear(SDIO, SDIO_INT_FLAG_IDMAERR);
        transerror = SD_DMA_ERROR;
        /* disable all the interrupts */
        sdio_interrupt_disable(SDIO, SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_DTEND |
                               SDIO_INT_TFH | SDIO_INT_RFH | SDIO_INT_TXURE | SDIO_INT_RXORE);
        sdio_trans_start_disable(SDIO);
        sdio_fifo_reset_enable(SDIO);
        sdio_fifo_reset_disable(SDIO);
        /* send CMD12 to stop data transfer in multipule blocks operation */
        transerror = sd_transfer_stop();
        sdio_flag_clear(SDIO, SDIO_FLAG_DTABORT);
        number_bytes = 0;

        sdio_idma_disable(SDIO);
        return transerror;
    }
    return transerror;
}

/*!
    \brief      select or deselect a card
    \param[in]  cardrca: the RCA of a card
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_card_select_deselect(uint16_t cardrca)
{
    sd_error_enum status = SD_OK;
    /* send CMD7(SELECT/DESELECT_CARD) to select or deselect the card */
    sdio_command_response_config(SDIO, SD_CMD_SELECT_DESELECT_CARD, (uint32_t)cardrca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
    sdio_csm_enable(SDIO);

    status = r1_error_check(SD_CMD_SELECT_DESELECT_CARD);
    return status;
}

/*!
    \brief      get the card status whose response format R1 contains a 32-bit field
    \param[in]  none
    \param[out] pcardstatus: a pointer that store card status
    \retval     sd_error_enum
*/
sd_error_enum sd_cardstatus_get(uint32_t *pcardstatus)
{
    sd_error_enum status = SD_OK;
    if(NULL == pcardstatus) {
        status = SD_PARAMETER_INVALID;
        return status;
    }

    /* send CMD13(SEND_STATUS), addressed card sends its status register */
    sdio_command_response_config(SDIO, SD_CMD_SEND_STATUS, (uint32_t)sd_rca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
    sdio_csm_enable(SDIO);
    /* check if some error occurs */
    status = r1_error_check(SD_CMD_SEND_STATUS);
    if(SD_OK != status) {
        return status;
    }

    *pcardstatus = sdio_response_get(SDIO, SDIO_RESPONSE0);
    return status;
}

/*!
    \brief      get SD card capacity
    \param[in]  none
    \param[out] none
    \retval     capacity of the card(KB)
*/
uint32_t sd_card_capacity_get(void)
{
    uint8_t tempbyte = 0U, devicesize_mult = 0U, readblklen = 0U;
    uint32_t capacity = 0U, devicesize = 0U;
    if((SDIO_STD_CAPACITY_SD_CARD_V1_1 == cardtype) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == cardtype)) {
        /* calculate the c_size(device size) */
        tempbyte = (uint8_t)((sd_csd[1] & SD_MASK_8_15BITS) >> 8U);
        devicesize |= ((uint32_t)tempbyte & 0x03U) << 10U;
        tempbyte = (uint8_t)(sd_csd[1] & SD_MASK_0_7BITS);
        devicesize |= (uint32_t)((uint32_t)tempbyte << 2U);
        tempbyte = (uint8_t)((sd_csd[2] & SD_MASK_24_31BITS) >> 24U);
        devicesize |= ((uint32_t)tempbyte & 0xC0U) >> 6U;

        /* calculate the c_size_mult(device size multiplier) */
        tempbyte = (uint8_t)((sd_csd[2] & SD_MASK_16_23BITS) >> 16U);
        devicesize_mult = (tempbyte & 0x03U) << 1U;
        tempbyte = (uint8_t)((sd_csd[2] & SD_MASK_8_15BITS) >> 8U);
        devicesize_mult |= (tempbyte & 0x80U) >> 7U;

        /* calculate the read_bl_len */
        tempbyte = (uint8_t)((sd_csd[1] & SD_MASK_16_23BITS) >> 16U);
        readblklen = tempbyte & 0x0FU;

        /* capacity = BLOCKNR*BLOCK_LEN, BLOCKNR = (C_SIZE+1)*MULT, MULT = 2^(C_SIZE_MULT+2), BLOCK_LEN = 2^READ_BL_LEN */
        capacity = (devicesize + 1U) * (1U << (devicesize_mult + 2U));
        capacity *= (1U << readblklen);

        /* change the unit of capacity to KByte */
        capacity /= 1024U;
    } else if(SDIO_HIGH_CAPACITY_SD_CARD == cardtype) {
        /* calculate the c_size */
        tempbyte = (uint8_t)(sd_csd[1] & SD_MASK_0_7BITS);
        devicesize = ((uint32_t)tempbyte & 0x3FU) << 16U;
        tempbyte = (uint8_t)((sd_csd[2] & SD_MASK_24_31BITS) >> 24U);
        devicesize |= (uint32_t)((uint32_t)tempbyte << 8U);
        tempbyte = (uint8_t)((sd_csd[2] & SD_MASK_16_23BITS) >> 16U);
        devicesize |= (uint32_t)tempbyte;

        /* capacity = (c_size+1)*512KByte */
        capacity = (devicesize + 1U) * 512U;
    } else {
        /* if else end */
    }
    return capacity;
}

/*!
    \brief      get the detailed information of the SD card based on received CID and CSD
    \param[in]  none
    \param[out] pcardinfo: a pointer that store the detailed card information
    \retval     sd_error_enum
*/
sd_error_enum sd_card_information_get(sd_card_info_struct *pcardinfo)
{
    sd_error_enum status = SD_OK;
    uint8_t tempbyte = 0U;

    if(NULL == pcardinfo) {
        status = SD_PARAMETER_INVALID;
        return status;
    }

    /* store the card type and RCA */
    pcardinfo->card_type = cardtype;
    pcardinfo->card_rca = sd_rca;

    /* CID byte 0 */
    tempbyte = (uint8_t)((sd_cid[0] & SD_MASK_24_31BITS) >> 24U);
    pcardinfo->card_cid.mid = tempbyte;

    /* CID byte 1 */
    tempbyte = (uint8_t)((sd_cid[0] & SD_MASK_16_23BITS) >> 16U);
    pcardinfo->card_cid.oid = (uint16_t)((uint16_t)tempbyte << 8U);

    /* CID byte 2 */
    tempbyte = (uint8_t)((sd_cid[0] & SD_MASK_8_15BITS) >> 8U);
    pcardinfo->card_cid.oid |= (uint16_t)tempbyte;

    /* CID byte 3 */
    tempbyte = (uint8_t)(sd_cid[0] & SD_MASK_0_7BITS);
    pcardinfo->card_cid.pnm0 = (uint32_t)((uint32_t)tempbyte << 24U);

    /* CID byte 4 */
    tempbyte = (uint8_t)((sd_cid[1] & SD_MASK_24_31BITS) >> 24U);
    pcardinfo->card_cid.pnm0 |= (uint32_t)((uint32_t)tempbyte << 16U);

    /* CID byte 5 */
    tempbyte = (uint8_t)((sd_cid[1] & SD_MASK_16_23BITS) >> 16U);
    pcardinfo->card_cid.pnm0 |= (uint32_t)((uint32_t)tempbyte << 8U);

    /* CID byte 6 */
    tempbyte = (uint8_t)((sd_cid[1] & SD_MASK_8_15BITS) >> 8U);
    pcardinfo->card_cid.pnm0 |= (uint32_t)(tempbyte);

    /* CID byte 7 */
    tempbyte = (uint8_t)(sd_cid[1] & SD_MASK_0_7BITS);
    pcardinfo->card_cid.pnm1 = tempbyte;

    /* CID byte 8 */
    tempbyte = (uint8_t)((sd_cid[2] & SD_MASK_24_31BITS) >> 24U);
    pcardinfo->card_cid.prv = tempbyte;

    /* CID byte 9 */
    tempbyte = (uint8_t)((sd_cid[2] & SD_MASK_16_23BITS) >> 16U);
    pcardinfo->card_cid.psn = (uint32_t)((uint32_t)tempbyte << 24U);

    /* CID byte 10 */
    tempbyte = (uint8_t)((sd_cid[2] & SD_MASK_8_15BITS) >> 8U);
    pcardinfo->card_cid.psn |= (uint32_t)((uint32_t)tempbyte << 16U);

    /* CID byte 11 */
    tempbyte = (uint8_t)(sd_cid[2] & SD_MASK_0_7BITS);
    pcardinfo->card_cid.psn |= (uint32_t)tempbyte;

    /* CID byte 12 */
    tempbyte = (uint8_t)((sd_cid[3] & SD_MASK_24_31BITS) >> 24U);
    pcardinfo->card_cid.psn |= (uint32_t)tempbyte;

    /* CID byte 13 */
    tempbyte = (uint8_t)((sd_cid[3] & SD_MASK_16_23BITS) >> 16U);
    pcardinfo->card_cid.mdt = ((uint16_t)tempbyte & 0x0FU) << 8U;

    /* CID byte 14 */
    tempbyte = (uint8_t)((sd_cid[3] & SD_MASK_8_15BITS) >> 8U);
    pcardinfo->card_cid.mdt |= (uint16_t)tempbyte;

    /* CID byte 15 */
    tempbyte = (uint8_t)(sd_cid[3] & SD_MASK_0_7BITS);
    pcardinfo->card_cid.cid_crc = (tempbyte & 0xFEU) >> 1U;

    /* CSD byte 0 */
    tempbyte = (uint8_t)((sd_csd[0] & SD_MASK_24_31BITS) >> 24U);
    pcardinfo->card_csd.csd_struct = (tempbyte & 0xC0U) >> 6U;

    /* CSD byte 1 */
    tempbyte = (uint8_t)((sd_csd[0] & SD_MASK_16_23BITS) >> 16U);
    pcardinfo->card_csd.taac = tempbyte;

    /* CSD byte 2 */
    tempbyte = (uint8_t)((sd_csd[0] & SD_MASK_8_15BITS) >> 8U);
    pcardinfo->card_csd.nsac = tempbyte;

    /* CSD byte 3 */
    tempbyte = (uint8_t)(sd_csd[0] & SD_MASK_0_7BITS);
    pcardinfo->card_csd.tran_speed = tempbyte;

    /* CSD byte 4 */
    tempbyte = (uint8_t)((sd_csd[1] & SD_MASK_24_31BITS) >> 24U);
    pcardinfo->card_csd.ccc = (uint16_t)((uint16_t)tempbyte << 4U);

    /* CSD byte 5 */
    tempbyte = (uint8_t)((sd_csd[1] & SD_MASK_16_23BITS) >> 16U);
    pcardinfo->card_csd.ccc |= ((uint16_t)tempbyte & 0xF0U) >> 4U;
    pcardinfo->card_csd.read_bl_len = tempbyte & 0x0FU;

    /* CSD byte 6 */
    tempbyte = (uint8_t)((sd_csd[1] & SD_MASK_8_15BITS) >> 8U);
    pcardinfo->card_csd.read_bl_partial = (tempbyte & 0x80U) >> 7U;
    pcardinfo->card_csd.write_blk_misalign = (tempbyte & 0x40U) >> 6U;
    pcardinfo->card_csd.read_blk_misalign = (tempbyte & 0x20U) >> 5U;
    pcardinfo->card_csd.dsp_imp = (tempbyte & 0x10U) >> 4U;

    if((SDIO_STD_CAPACITY_SD_CARD_V1_1 == cardtype) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == cardtype)) {
        /* card is SDSC card, CSD version 1.0 */
        pcardinfo->card_csd.c_size = ((uint32_t)tempbyte & 0x03U) << 10U;

        /* CSD byte 7 */
        tempbyte = (uint8_t)(sd_csd[1] & SD_MASK_0_7BITS);
        pcardinfo->card_csd.c_size |= (uint32_t)((uint32_t)tempbyte << 2U);

        /* CSD byte 8 */
        tempbyte = (uint8_t)((sd_csd[2] & SD_MASK_24_31BITS) >> 24U);
        pcardinfo->card_csd.c_size |= ((uint32_t)tempbyte & 0xC0U) >> 6U;
        pcardinfo->card_csd.vdd_r_curr_min = (tempbyte & 0x38U) >> 3U;
        pcardinfo->card_csd.vdd_r_curr_max = tempbyte & 0x07U;

        /* CSD byte 9 */
        tempbyte = (uint8_t)((sd_csd[2] & SD_MASK_16_23BITS) >> 16U);
        pcardinfo->card_csd.vdd_w_curr_min = (tempbyte & 0xE0U) >> 5U;
        pcardinfo->card_csd.vdd_w_curr_max = (tempbyte & 0x1CU) >> 2U;
        pcardinfo->card_csd.c_size_mult = (tempbyte & 0x03U) << 1U;

        /* CSD byte 10 */
        tempbyte = (uint8_t)((sd_csd[2] & SD_MASK_8_15BITS) >> 8U);
        pcardinfo->card_csd.c_size_mult |= (tempbyte & 0x80U) >> 7U;

        /* calculate the card block size and capacity */
        pcardinfo->card_blocksize = 1U << (pcardinfo->card_csd.read_bl_len);
        pcardinfo->card_capacity = pcardinfo->card_csd.c_size + 1U;
        pcardinfo->card_capacity *= (1U << (pcardinfo->card_csd.c_size_mult + 2U));
        pcardinfo->card_capacity *= pcardinfo->card_blocksize;
    } else if(SDIO_HIGH_CAPACITY_SD_CARD == cardtype) {
        /* card is SDHC card, CSD version 2.0 */
        /* CSD byte 7 */
        tempbyte = (uint8_t)(sd_csd[1] & SD_MASK_0_7BITS);
        pcardinfo->card_csd.c_size = ((uint32_t)tempbyte & 0x3FU) << 16U;

        /* CSD byte 8 */
        tempbyte = (uint8_t)((sd_csd[2] & SD_MASK_24_31BITS) >> 24U);
        pcardinfo->card_csd.c_size |= (uint32_t)((uint32_t)tempbyte << 8U);

        /* CSD byte 9 */
        tempbyte = (uint8_t)((sd_csd[2] & SD_MASK_16_23BITS) >> 16U);
        pcardinfo->card_csd.c_size |= (uint32_t)tempbyte;

        /* calculate the card block size and capacity */
        pcardinfo->card_blocksize = 512U;
        pcardinfo->card_capacity = (pcardinfo->card_csd.c_size + 1U) * 512U * 1024U;
    } else {
        /* if else end */
    }

    pcardinfo->card_csd.erase_blk_en = (tempbyte & 0x40U) >> 6U;
    pcardinfo->card_csd.sector_size = (tempbyte & 0x3FU) << 1U;

    /* CSD byte 11 */
    tempbyte = (uint8_t)(sd_csd[2] & SD_MASK_0_7BITS);
    pcardinfo->card_csd.sector_size |= (tempbyte & 0x80U) >> 7U;
    pcardinfo->card_csd.wp_grp_size = (tempbyte & 0x7FU);

    /* CSD byte 12 */
    tempbyte = (uint8_t)((sd_csd[3] & SD_MASK_24_31BITS) >> 24U);
    pcardinfo->card_csd.wp_grp_enable = (tempbyte & 0x80U) >> 7U;
    pcardinfo->card_csd.r2w_factor = (tempbyte & 0x1CU) >> 2U;
    pcardinfo->card_csd.write_bl_len = (tempbyte & 0x03U) << 2U;

    /* CSD byte 13 */
    tempbyte = (uint8_t)((sd_csd[3] & SD_MASK_16_23BITS) >> 16U);
    pcardinfo->card_csd.write_bl_len |= (tempbyte & 0xC0U) >> 6U;
    pcardinfo->card_csd.write_bl_partial = (tempbyte & 0x20U) >> 5U;

    /* CSD byte 14 */
    tempbyte = (uint8_t)((sd_csd[3] & SD_MASK_8_15BITS) >> 8U);
    pcardinfo->card_csd.file_format_grp = (tempbyte & 0x80U) >> 7U;
    pcardinfo->card_csd.copy_flag = (tempbyte & 0x40U) >> 6U;
    pcardinfo->card_csd.perm_write_protect = (tempbyte & 0x20U) >> 5U;
    pcardinfo->card_csd.tmp_write_protect = (tempbyte & 0x10U) >> 4U;
    pcardinfo->card_csd.file_format = (tempbyte & 0x0CU) >> 2U;

    /* CSD byte 15 */
    tempbyte = (uint8_t)(sd_csd[3] & SD_MASK_0_7BITS);
    pcardinfo->card_csd.csd_crc = (tempbyte & 0xFEU) >> 1U;

    return status;
}

/*!
    \brief      check if the command sent error occurs
    \param[in]  none
    \param[out] none
    \retval     sd_error_enum
*/
static sd_error_enum cmdsent_error_check(void)
{
    sd_error_enum status = SD_OK;
    uint32_t timeout = 100000U;
    /* check command sent flag */
    while((RESET == sdio_flag_get(SDIO, SDIO_FLAG_CMDSEND)) && (timeout > 0U)) {
        --timeout;
    }
    /* command response is timeout */
    if(0U == timeout) {
        status = SD_CMD_RESP_TIMEOUT;
        return status;
    }
    /* if the command is sent, clear the CMD_FLAGS flags */
    sdio_flag_clear(SDIO, SDIO_MASK_CMD_FLAGS);
    return status;
}

/*!
    \brief      check if error type for R1 response
    \param[in]  resp: content of response
    \param[out] none
    \retval     sd_error_enum
*/
static sd_error_enum r1_error_type_check(uint32_t resp)
{
    sd_error_enum status = SD_ERROR;
    /* check which error occurs */
    if(resp & SD_R1_OUT_OF_RANGE) {
        status = SD_OUT_OF_RANGE;
    } else if(resp & SD_R1_ADDRESS_ERROR) {
        status = SD_ADDRESS_ERROR;
    } else if(resp & SD_R1_BLOCK_LEN_ERROR) {
        status = SD_BLOCK_LEN_ERROR;
    } else if(resp & SD_R1_ERASE_SEQ_ERROR) {
        status = SD_ERASE_SEQ_ERROR;
    } else if(resp & SD_R1_ERASE_PARAM) {
        status = SD_ERASE_PARAM;
    } else if(resp & SD_R1_WP_VIOLATION) {
        status = SD_WP_VIOLATION;
    } else if(resp & SD_R1_LOCK_UNLOCK_FAILED) {
        status = SD_LOCK_UNLOCK_FAILED;
    } else if(resp & SD_R1_COM_CRC_ERROR) {
        status = SD_COM_CRC_ERROR;
    } else if(resp & SD_R1_ILLEGAL_COMMAND) {
        status = SD_ILLEGAL_COMMAND;
    } else if(resp & SD_R1_CARD_ECC_FAILED) {
        status = SD_CARD_ECC_FAILED;
    } else if(resp & SD_R1_CC_ERROR) {
        status = SD_CC_ERROR;
    } else if(resp & SD_R1_GENERAL_UNKNOWN_ERROR) {
        status = SD_GENERAL_UNKNOWN_ERROR;
    } else if(resp & SD_R1_CSD_OVERWRITE) {
        status = SD_CSD_OVERWRITE;
    } else if(resp & SD_R1_WP_ERASE_SKIP) {
        status = SD_WP_ERASE_SKIP;
    } else if(resp & SD_R1_CARD_ECC_DISABLED) {
        status = SD_CARD_ECC_DISABLED;
    } else if(resp & SD_R1_ERASE_RESET) {
        status = SD_ERASE_RESET;
    } else if(resp & SD_R1_AKE_SEQ_ERROR) {
        status = SD_AKE_SEQ_ERROR;
    } else {
        /*no todo,  if else end */
    }
    return status;
}

/*!
    \brief      check if error occurs for R1 response
    \param[in]  cmdindex: the index of command
    \param[out] none
    \retval     sd_error_enum
*/
static sd_error_enum r1_error_check(uint8_t cmdindex)
{
    sd_error_enum status = SD_OK;
    uint32_t reg_status = 0U, resp_r1 = 0U;

    /* store the content of SDIO_STAT */
    reg_status = SDIO_STAT(SDIO);
    while(!(reg_status & (SDIO_FLAG_CCRCERR | SDIO_FLAG_CMDTMOUT | SDIO_FLAG_CMDRECV))) {
        reg_status = SDIO_STAT(SDIO);
    }
    /* check whether an error or timeout occurs or command response received */
    if(reg_status & SDIO_FLAG_CCRCERR) {
        status = SD_CMD_CRC_ERROR;
        sdio_flag_clear(SDIO, SDIO_FLAG_CCRCERR);
        return status;
    } else if(reg_status & SDIO_FLAG_CMDTMOUT) {
        status = SD_CMD_RESP_TIMEOUT;
        sdio_flag_clear(SDIO, SDIO_FLAG_CMDTMOUT);
        return status;
    } else {
        /* if else end */
    }

    /* check whether the last response command index is the desired one */
    if(sdio_command_index_get(SDIO) != cmdindex) {
        status = SD_ILLEGAL_COMMAND;
        return status;
    }
    /* clear all the CMD_FLAGS flags */
    sdio_flag_clear(SDIO, SDIO_MASK_CMD_FLAGS);
    /* get the SDIO response register 0 for checking */
    resp_r1 = sdio_response_get(SDIO, SDIO_RESPONSE0);
    if(SD_ALLZERO == (resp_r1 & SD_R1_ERROR_BITS)) {
        /* no error occurs, return SD_OK */
        status = SD_OK;
        return status;
    }

    /* if some error occurs, return the error type */
    status = r1_error_type_check(resp_r1);
    return status;
}

/*!
    \brief      check if error occurs for R2 response
    \param[in]  none
    \param[out] none
    \retval     sd_error_enum
*/
static sd_error_enum r2_error_check(void)
{
    sd_error_enum status = SD_OK;
    uint32_t reg_status = 0U;

    /* store the content of SDIO_STAT */
    reg_status = SDIO_STAT(SDIO);
    while(!(reg_status & (SDIO_FLAG_CCRCERR | SDIO_FLAG_CMDTMOUT | SDIO_FLAG_CMDRECV))) {
        reg_status = SDIO_STAT(SDIO);
    }
    /* check whether an error or timeout occurs or command response received */
    if(reg_status & SDIO_FLAG_CCRCERR) {
        status = SD_CMD_CRC_ERROR;
        sdio_flag_clear(SDIO, SDIO_FLAG_CCRCERR);
        return status;
    } else if(reg_status & SDIO_FLAG_CMDTMOUT) {
        status = SD_CMD_RESP_TIMEOUT;
        sdio_flag_clear(SDIO, SDIO_FLAG_CMDTMOUT);
        return status;
    } else {
        /* if else end */
    }
    /* clear all the CMD_FLAGS flags */
    sdio_flag_clear(SDIO, SDIO_MASK_CMD_FLAGS);

    return status;
}


/*!
    \brief      check if error occurs for R3 response
    \param[in]  none
    \param[out] none
    \retval     sd_error_enum
*/
static sd_error_enum r3_error_check(void)
{
    sd_error_enum status = SD_OK;
    uint32_t reg_status = 0U;

    /* store the content of SDIO_STAT */
    reg_status = SDIO_STAT(SDIO);
    while(!(reg_status & (SDIO_FLAG_CCRCERR | SDIO_FLAG_CMDTMOUT | SDIO_FLAG_CMDRECV))) {
        reg_status = SDIO_STAT(SDIO);
    }
    if(reg_status & SDIO_FLAG_CMDTMOUT) {
        status = SD_CMD_RESP_TIMEOUT;
        sdio_flag_clear(SDIO, SDIO_FLAG_CMDTMOUT);
        return status;
    }
    /* clear all the CMD_FLAGS flags */
    sdio_flag_clear(SDIO, SDIO_MASK_CMD_FLAGS);
    return status;
}

/*!
    \brief      check if error occurs for R6 response
    \param[in]  cmdindex: the index of command
    \param[out] prca: a pointer that store the RCA of card
    \retval     sd_error_enum
*/
static sd_error_enum r6_error_check(uint8_t cmdindex, uint16_t *prca)
{
    sd_error_enum status = SD_OK;
    uint32_t reg_status = 0U, response = 0U;

    /* store the content of SDIO_STAT */
    reg_status = SDIO_STAT(SDIO);
    while(!(reg_status & (SDIO_FLAG_CCRCERR | SDIO_FLAG_CMDTMOUT | SDIO_FLAG_CMDRECV))) {
        reg_status = SDIO_STAT(SDIO);
    }
    /* check whether an error or timeout occurs or command response received */
    if(reg_status & SDIO_FLAG_CCRCERR) {
        status = SD_CMD_CRC_ERROR;
        sdio_flag_clear(SDIO, SDIO_FLAG_CCRCERR);
        return status;
    } else if(reg_status & SDIO_FLAG_CMDTMOUT) {
        status = SD_CMD_RESP_TIMEOUT;
        sdio_flag_clear(SDIO, SDIO_FLAG_CMDTMOUT);
        return status;
    } else {
        /* if else end */
    }

    /* check whether the last response command index is the desired one */
    if(sdio_command_index_get(SDIO) != cmdindex) {
        status = SD_ILLEGAL_COMMAND;
        return status;
    }
    /* clear all the CMD_FLAGS flags */
    sdio_flag_clear(SDIO, SDIO_MASK_CMD_FLAGS);
    /* get the SDIO response register 0 for checking */
    response = sdio_response_get(SDIO, SDIO_RESPONSE0);

    if(SD_ALLZERO == (response & (SD_R6_COM_CRC_ERROR | SD_R6_ILLEGAL_COMMAND | SD_R6_GENERAL_UNKNOWN_ERROR))) {
        *prca = (uint16_t)(response >> 16U);
        return status;
    }
    /* if some error occurs, return the error type */
    if(response & SD_R6_COM_CRC_ERROR) {
        status = SD_COM_CRC_ERROR;
    } else if(response & SD_R6_ILLEGAL_COMMAND) {
        status = SD_ILLEGAL_COMMAND;
    } else if(response & SD_R6_GENERAL_UNKNOWN_ERROR) {
        status = SD_GENERAL_UNKNOWN_ERROR;
    } else {
        /* if else end */
    }
    return status;
}


/*!
    \brief      check if error occurs for R7 response
    \param[in]  none
    \param[out] none
    \retval     sd_error_enum
*/
static sd_error_enum r7_error_check(void)
{
    sd_error_enum status = SD_ERROR;
    uint32_t reg_status = 0U, timeout = 100000U;

    /* store the content of SDIO_STAT */
    reg_status = SDIO_STAT(SDIO);
    while(((reg_status & (SDIO_FLAG_CCRCERR | SDIO_FLAG_CMDTMOUT | SDIO_FLAG_CMDRECV)) == 0U) && (timeout > 0U)) {
        reg_status = SDIO_STAT(SDIO);
        --timeout;
    }

    /* check the flags */
    if((reg_status & SDIO_FLAG_CMDTMOUT) || (0U == timeout)) {
        status = SD_CMD_RESP_TIMEOUT;
        sdio_flag_clear(SDIO, SDIO_FLAG_CMDTMOUT);
        return status;
    }
    if(reg_status & SDIO_FLAG_CMDRECV) {
        status = SD_OK;
        sdio_flag_clear(SDIO, SDIO_FLAG_CMDRECV);
        return status;
    }
    return status;
}

/*!
    \brief      get the state which the card is in
    \param[in]  none
    \param[out] pcardstate: a pointer that store the card state
      \arg        SD_CARDSTATE_IDLE: card is in idle state
      \arg        SD_CARDSTATE_READY: card is in ready state
      \arg        SD_CARDSTATE_IDENTIFICAT: card is in identificat state
      \arg        SD_CARDSTATE_STANDBY: card is in standby state
      \arg        SD_CARDSTATE_TRANSFER: card is in transfer state
      \arg        SD_CARDSTATE_DATA: card is in data state
      \arg        SD_CARDSTATE_RECEIVING: card is in receiving state
      \arg        SD_CARDSTATE_PROGRAMMING: card is in programming state
      \arg        SD_CARDSTATE_DISCONNECT: card is in disconnect state
      \arg        SD_CARDSTATE_LOCKED: card is in locked state
    \retval     sd_error_enum
*/
static sd_error_enum sd_card_state_get(uint8_t *pcardstate)
{
    sd_error_enum status = SD_OK;
    __IO uint32_t reg_status = 0U, response = 0U;

    /* send CMD13(SEND_STATUS), addressed card sends its status register */
    sdio_command_response_config(SDIO, SD_CMD_SEND_STATUS, (uint32_t)sd_rca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
    sdio_csm_enable(SDIO);

    /* store the content of SDIO_STAT */
    reg_status = SDIO_STAT(SDIO);
    while(!(reg_status & (SDIO_FLAG_CCRCERR | SDIO_FLAG_CMDTMOUT | SDIO_FLAG_CMDRECV))) {
        reg_status = SDIO_STAT(SDIO);
    }
    /* check whether an error or timeout occurs or command response received */
    if(reg_status & SDIO_FLAG_CCRCERR) {
        status = SD_CMD_CRC_ERROR;
        sdio_flag_clear(SDIO, SDIO_FLAG_CCRCERR);
        return status;
    } else if(reg_status & SDIO_FLAG_CMDTMOUT) {
        status = SD_CMD_RESP_TIMEOUT;
        sdio_flag_clear(SDIO, SDIO_FLAG_CMDTMOUT);
        return status;
    } else {
        /* if else end */
    }

    /* command response received, store the response command index */
    reg_status = (uint32_t)sdio_command_index_get(SDIO);
    if(reg_status != (uint32_t)SD_CMD_SEND_STATUS) {
        status = SD_ILLEGAL_COMMAND;
        return status;
    }
    /* clear all the SDIO_INTC flags */
    sdio_flag_clear(SDIO, SDIO_MASK_INTC_FLAGS);
    /* get the SDIO response register 0 for checking */
    response = sdio_response_get(SDIO, SDIO_RESPONSE0);
    *pcardstate = (uint8_t)((response >> 9U) & 0x0000000FU);

    if(SD_ALLZERO == (response & SD_R1_ERROR_BITS)) {
        /* no error occurs, return SD_OK */
        status = SD_OK;
        return status;
    }

    /* if some error occurs, return the error type */
    status = r1_error_type_check(response);
    return status;
}

/*!
    \brief      stop an ongoing data transfer
    \param[in]  none
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_transfer_stop(void)
{
    sd_error_enum status = SD_OK;
    /* send CMD12(STOP_TRANSMISSION) to stop transmission */
    sdio_command_response_config(SDIO, SD_CMD_STOP_TRANSMISSION, (uint32_t)0x0U, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
    sdio_trans_stop_enable(SDIO);
    sdio_csm_enable(SDIO);
    /* check if some error occurs */
    status = r1_error_check(SD_CMD_STOP_TRANSMISSION);
    sdio_trans_stop_disable(SDIO);
    return status;
}

/*!
    \brief      lock or unlock a card
    \param[in]  lockstate: the lock state
      \arg        SD_LOCK: lock the SD card
      \arg        SD_UNLOCK: unlock the SD card
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_lock_unlock(uint8_t lockstate)
{
    sd_error_enum status = SD_OK;
    uint8_t cardstate = 0U, tempbyte = 0U;
    uint32_t pwd1 = 0U, pwd2 = 0U, response = 0U, timeout = 0U;
    uint16_t tempccc = 0U;

    /* get the card command classes from CSD */
    tempbyte = (uint8_t)((sd_csd[1] & SD_MASK_24_31BITS) >> 24U);
    tempccc = ((uint16_t)tempbyte << 4U);
    tempbyte = (uint8_t)((sd_csd[1] & SD_MASK_16_23BITS) >> 16U);
    tempccc |= (((uint16_t)tempbyte & 0xF0U) >> 4U);

    if(0U == (tempccc & SD_CCC_LOCK_CARD)) {
        /* don't support the lock command */
        status = SD_FUNCTION_UNSUPPORTED;
        return status;
    }
    /* password pattern */
    pwd1 = (0x01020600U | lockstate);
    pwd2 = 0x03040506U;

    /* clear all DSM configuration */
    sdio_data_config(SDIO, 0U, 0U, SDIO_DATABLOCKSIZE_1BYTE);
    sdio_data_transfer_config(SDIO, SDIO_TRANSMODE_BLOCKCOUNT, SDIO_TRANSDIRECTION_TOCARD);
    sdio_dsm_disable(SDIO);
    sdio_idma_disable(SDIO);

    /* send CMD16(SET_BLOCKLEN) to set the block length */
    sdio_command_response_config(SDIO, SD_CMD_SET_BLOCKLEN, (uint32_t)8U, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
    sdio_csm_enable(SDIO);
    /* check if some error occurs */
    status = r1_error_check(SD_CMD_SET_BLOCKLEN);
    if(SD_OK != status) {
        return status;
    }

    /* send CMD13(SEND_STATUS), addressed card sends its status register */
    sdio_command_response_config(SDIO, SD_CMD_SEND_STATUS, (uint32_t)sd_rca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
    sdio_csm_enable(SDIO);
    /* check if some error occurs */
    status = r1_error_check(SD_CMD_SEND_STATUS);
    if(SD_OK != status) {
        return status;
    }

    response = sdio_response_get(SDIO, SDIO_RESPONSE0);
    timeout = 100000U;
    while((0U == (response & SD_R1_READY_FOR_DATA)) && (timeout > 0U)) {
        /* continue to send CMD13 to polling the state of card until buffer empty or timeout */
        --timeout;
        /* send CMD13(SEND_STATUS), addressed card sends its status registers */
        sdio_command_response_config(SDIO, SD_CMD_SEND_STATUS, (uint32_t)sd_rca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
        sdio_csm_enable(SDIO);
        /* check if some error occurs */
        status = r1_error_check(SD_CMD_SEND_STATUS);
        if(SD_OK != status) {
            return status;
        }
        response = sdio_response_get(SDIO, SDIO_RESPONSE0);
    }
    if(0U == timeout) {
        status = SD_ERROR;
        return status;
    }

    /* send CMD42(LOCK_UNLOCK) to set/reset the password or lock/unlock the card */
    sdio_command_response_config(SDIO, SD_CMD_LOCK_UNLOCK, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
    sdio_csm_enable(SDIO);
    /* check if some error occurs */
    status = r1_error_check(SD_CMD_LOCK_UNLOCK);
    if(SD_OK != status) {
        return status;
    }

    response = sdio_response_get(SDIO, SDIO_RESPONSE0);

    /* configure the SDIO data transmisson */
    sdio_data_config(SDIO, SD_DATATIMEOUT, (uint32_t)8, SDIO_DATABLOCKSIZE_8BYTES);
    sdio_data_transfer_config(SDIO, SDIO_TRANSMODE_BLOCKCOUNT, SDIO_TRANSDIRECTION_TOCARD);
    sdio_dsm_enable(SDIO);

    /* write password pattern */
    sdio_data_write(SDIO, pwd1);
    sdio_data_write(SDIO, pwd2);

    /* whether some error occurs and return it */
    if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_DTCRCERR)) {
        status = SD_DATA_CRC_ERROR;
        sdio_flag_clear(SDIO, SDIO_FLAG_DTCRCERR);
        return status;
    } else if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_DTTMOUT)) {
        status = SD_DATA_TIMEOUT;
        sdio_flag_clear(SDIO, SDIO_FLAG_DTTMOUT);
        return status;
    } else if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_TXURE)) {
        status = SD_TX_UNDERRUN_ERROR;
        sdio_flag_clear(SDIO, SDIO_FLAG_TXURE);
        return status;
    } else {
        /* if else end */
    }

    /* clear the SDIO_INTC flags */
    sdio_flag_clear(SDIO, SDIO_MASK_INTC_FLAGS);
    /* get the card state and wait the card is out of programming and receiving state */
    status = sd_card_state_get(&cardstate);
    while((SD_OK == status) && ((SD_CARDSTATE_PROGRAMMING == cardstate) || (SD_CARDSTATE_RECEIVING == cardstate))) {
        status = sd_card_state_get(&cardstate);
    }
    return status;
}

/*!
    \brief      configure the bus width mode
    \param[in]  buswidth: the bus width
      \arg        SD_BUS_WIDTH_1BIT: 1-bit bus width
      \arg        SD_BUS_WIDTH_4BIT: 4-bit bus width
    \param[out] none
    \retval     sd_error_enum
*/
static sd_error_enum sd_bus_width_config(uint32_t buswidth)
{
    sd_error_enum status = SD_OK;
    /* check whether the card is locked */
    if(sdio_response_get(SDIO, SDIO_RESPONSE0) & SD_CARDSTATE_LOCKED) {
        status = SD_LOCK_UNLOCK_FAILED;
        return status;
    }
    /* get the SCR register */
    status = sd_scr_get(sd_rca, sd_scr);
    if(SD_OK != status) {
        return status;
    }

    if(SD_BUS_WIDTH_1BIT == buswidth) {
        if(SD_ALLZERO != (sd_scr[1] & buswidth)) {
            /* send CMD55(APP_CMD) to indicate next command is application specific command */
            sdio_command_response_config(SDIO, SD_CMD_APP_CMD, (uint32_t)sd_rca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
            sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
            sdio_csm_enable(SDIO);
            /* check if some error occurs */
            status = r1_error_check(SD_CMD_APP_CMD);
            if(SD_OK != status) {
                return status;
            }

            /* send ACMD6(SET_BUS_WIDTH) to define the data bus width */
            sdio_command_response_config(SDIO, SD_APPCMD_SET_BUS_WIDTH, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
            sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
            sdio_csm_enable(SDIO);
            /* check if some error occurs */
            status = r1_error_check(SD_APPCMD_SET_BUS_WIDTH);
            if(SD_OK != status) {
                return status;
            }
        } else {
            status = SD_OPERATION_IMPROPER;
        }
        return status;
    } else if(SD_BUS_WIDTH_4BIT == buswidth) {
        if(SD_ALLZERO != (sd_scr[1] & buswidth)) {
            /* send CMD55(APP_CMD) to indicate next command is application specific command */
            sdio_command_response_config(SDIO, SD_CMD_APP_CMD, (uint32_t)sd_rca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
            sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
            sdio_csm_enable(SDIO);
            /* check if some error occurs */
            status = r1_error_check(SD_CMD_APP_CMD);
            if(SD_OK != status) {
                return status;
            }

            /* send ACMD6(SET_BUS_WIDTH) to define the data bus width */
            sdio_command_response_config(SDIO, SD_APPCMD_SET_BUS_WIDTH, (uint32_t)0x2U, SDIO_RESPONSETYPE_SHORT);
            sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
            sdio_csm_enable(SDIO);
            /* check if some error occurs */
            status = r1_error_check(SD_APPCMD_SET_BUS_WIDTH);
            if(SD_OK != status) {
                return status;
            }
        } else {
            status = SD_OPERATION_IMPROPER;
        }
        return status;
    } else {
        status = SD_PARAMETER_INVALID;
        return status;
    }
}

/*!
    \brief      get the SCR of corresponding card
    \param[in]  rca: RCA of a card
    \param[out] pscr: a pointer that store the SCR content
    \retval     sd_error_enum
*/
static sd_error_enum sd_scr_get(uint16_t rca, uint32_t *pscr)
{
    uint32_t temp_scr[2] = {0, 0}, idx_scr = 0U;
    sd_error_enum status = SD_OK;
    /* send CMD16(SET_BLOCKLEN) to set block length */
    sdio_command_response_config(SDIO, SD_CMD_SET_BLOCKLEN, (uint32_t)8U, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
    sdio_csm_enable(SDIO);
    /* check if some error occurs */
    status = r1_error_check(SD_CMD_SET_BLOCKLEN);
    if(SD_OK != status) {
        return status;
    }

    /* send CMD55(APP_CMD) to indicate next command is application specific command */
    sdio_command_response_config(SDIO, SD_CMD_APP_CMD, (uint32_t)rca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
    sdio_csm_enable(SDIO);
    /* check if some error occurs */
    status = r1_error_check(SD_CMD_APP_CMD);
    if(SD_OK != status) {
        return status;
    }

    /* configure SDIO data */
    sdio_data_config(SDIO, SD_DATATIMEOUT, (uint32_t)8, SDIO_DATABLOCKSIZE_8BYTES);
    sdio_data_transfer_config(SDIO, SDIO_TRANSMODE_BLOCKCOUNT, SDIO_TRANSDIRECTION_TOSDIO);
    sdio_dsm_enable(SDIO);

    /* send ACMD51(SEND_SCR) to read the SD configuration register */
    sdio_command_response_config(SDIO, SD_APPCMD_SEND_SCR, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO, SDIO_WAITTYPE_NO);
    sdio_csm_enable(SDIO);
    /* check if some error occurs */
    status = r1_error_check(SD_APPCMD_SEND_SCR);
    if(SD_OK != status) {
        return status;
    }

    /* store the received SCR */
    while(!sdio_flag_get(SDIO, SDIO_FLAG_DTCRCERR | SDIO_FLAG_DTTMOUT | SDIO_FLAG_RXORE | SDIO_FLAG_DTBLKEND | SDIO_FLAG_DTEND)) {
        if((SET != sdio_flag_get(SDIO, SDIO_FLAG_RFE)) && (SET == sdio_flag_get(SDIO, SDIO_FLAG_DATSTA))) {
            *(temp_scr + idx_scr) = sdio_data_read(SDIO);
            ++idx_scr;
        }
    }

    /* check whether some error occurs */
    if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_DTCRCERR)) {
        status = SD_DATA_CRC_ERROR;
        sdio_flag_clear(SDIO, SDIO_FLAG_DTCRCERR);
        return status;
    } else if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_DTTMOUT)) {
        status = SD_DATA_TIMEOUT;
        sdio_flag_clear(SDIO, SDIO_FLAG_DTTMOUT);
        return status;
    } else if(RESET != sdio_flag_get(SDIO, SDIO_FLAG_RXORE)) {
        status = SD_RX_OVERRUN_ERROR;
        sdio_flag_clear(SDIO, SDIO_FLAG_RXORE);
        return status;
    } else {
        /* if else end */
    }

    /* clear all the SDIO_INTC flags */
    sdio_flag_clear(SDIO, SDIO_MASK_INTC_FLAGS);
    /* readjust the temp SCR value */
    *(pscr) = ((temp_scr[1] & SD_MASK_0_7BITS) << 24U) | ((temp_scr[1] & SD_MASK_8_15BITS) << 8U) |
              ((temp_scr[1] & SD_MASK_16_23BITS) >> 8U) | ((temp_scr[1] & SD_MASK_24_31BITS) >> 24U);
    *(pscr + 1U) = ((temp_scr[0] & SD_MASK_0_7BITS) << 24U) | ((temp_scr[0] & SD_MASK_8_15BITS) << 8U) |
                   ((temp_scr[0] & SD_MASK_16_23BITS) >> 8U) | ((temp_scr[0] & SD_MASK_24_31BITS) >> 24U);
    return status;
}

/*!
    \brief      get the data block size
    \param[in]  bytesnumber: the number of bytes
    \param[out] none
    \retval     data block size
      \arg        SDIO_DATABLOCKSIZE_1BYTE: block size = 1 byte
      \arg        SDIO_DATABLOCKSIZE_2BYTES: block size = 2 bytes
      \arg        SDIO_DATABLOCKSIZE_4BYTES: block size = 4 bytes
      \arg        SDIO_DATABLOCKSIZE_8BYTES: block size = 8 bytes
      \arg        SDIO_DATABLOCKSIZE_16BYTES: block size = 16 bytes
      \arg        SDIO_DATABLOCKSIZE_32BYTES: block size = 32 bytes
      \arg        SDIO_DATABLOCKSIZE_64BYTES: block size = 64 bytes
      \arg        SDIO_DATABLOCKSIZE_128BYTES: block size = 128 bytes
      \arg        SDIO_DATABLOCKSIZE_256BYTES: block size = 256 bytes
      \arg        SDIO_DATABLOCKSIZE_512BYTES: block size = 512 bytes
      \arg        SDIO_DATABLOCKSIZE_1024BYTES: block size = 1024 bytes
      \arg        SDIO_DATABLOCKSIZE_2048BYTES: block size = 2048 bytes
      \arg        SDIO_DATABLOCKSIZE_4096BYTES: block size = 4096 bytes
      \arg        SDIO_DATABLOCKSIZE_8192BYTES: block size = 8192 bytes
      \arg        SDIO_DATABLOCKSIZE_16384BYTES: block size = 16384 bytes
*/
static uint32_t sd_datablocksize_get(uint16_t bytesnumber)
{
    uint8_t exp_val = 0U;
    /* calculate the exponent of 2 */
    while(1U != bytesnumber) {
        bytesnumber >>= 1U;
        ++exp_val;
    }
    return DATACTL_BLKSZ(exp_val);
}

/*!
    \brief      configure the GPIO of SDIO interface
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void gpio_config(void)
{
    /* configure the SDIO_DAT0(PB13), SDIO_DAT1(PC9), SDIO_DAT2(PC10), SDIO_DAT3(PC11), SDIO_CLK(PC12) and SDIO_CMD(PD2) */
    gpio_af_set(GPIOB, GPIO_AF_12, GPIO_PIN_13);
    gpio_af_set(GPIOC, GPIO_AF_12, GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);
    gpio_af_set(GPIOD, GPIO_AF_12, GPIO_PIN_2);

    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_13);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11);

    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_13);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11);

    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_12);

    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_2);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_2);
}

/*!
    \brief      configure the RCU of SDIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void rcu_config(void)
{
    /* SDIO clock */
    /* configure the pll1 input and output clock range */
    rcu_pll_input_output_clock_range_config(IDX_PLL1, RCU_PLL1RNG_4M_8M, RCU_PLL1VCO_192M_836M);
    /* configure the PLL1 clock: CK_PLL1P/CK_PLL1Q/CK_PLL1R = HXTAL_VALUE / 5 * 40 / 1 */
    /* SDIO clock 400M */
    rcu_pll1_config(5, 160, 1, 2, 2);
    /* enable PLL1R clock output */
    rcu_pll_clock_output_enable(RCU_PLL1R);
    /* enable PLL1 clock */
    rcu_osci_on(RCU_PLL1_CK);

    if(ERROR == rcu_osci_stab_wait(RCU_PLL1_CK)) {
        while(1) {
        }
    }
    
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_sdio_clock_config(IDX_SDIO0, RCU_SDIO0SRC_PLL1R);
    rcu_periph_clock_enable(RCU_SDIO0);
}

/*!
    \brief      configure the IDMA
    \param[in]  srcbuf: a pointer point to a buffer which will be transferred
    \param[in]  bufsize: the size of buffer(not used in flow controller is peripheral)
    \param[out] none
    \retval     none
*/
static void dma_config(uint32_t *srcbuf, uint32_t bufsize)
{
    sdio_idma_set(SDIO, SDIO_IDMA_SINGLE_BUFFER, bufsize);
    sdio_idma_buffer0_address_set(SDIO, (uint32_t)srcbuf);
}

