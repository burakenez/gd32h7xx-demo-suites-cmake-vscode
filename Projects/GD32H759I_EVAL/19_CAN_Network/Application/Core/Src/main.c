/*!
    \file    main.c
    \brief   CAN network demo

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

#include "gd32h7xx.h"
#include <stdio.h>
#include "gd32h759i_eval.h"
#include "systick.h"
#include "string.h"

#define  BUFFER_SIZE    8U

const uint8_t tx_data[BUFFER_SIZE] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8};
uint8_t rx_data[BUFFER_SIZE];

can_mailbox_descriptor_struct transmit_message;
can_mailbox_descriptor_struct receive_message;
FlagStatus can1_receive_flag;

void can_gpio_config(void);
void bsp_board_config(void);
void can_config(void);
void communication_check(void);
void cache_enable(void);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    uint8_t i = 0U;
    can_struct_para_init(CAN_MDSC_STRUCT, &transmit_message);
    can_struct_para_init(CAN_MDSC_STRUCT, &receive_message);

    /* enable the CPU Cache */
    cache_enable();
    /* configure systick */
    systick_config();
    /* configure board */
    bsp_board_config();
    /* configure GPIO */
    can_gpio_config();
    /* initialize CAN and filter */
    can_config();

    printf("\r\ncommunication test CAN1 and CAN2, please press WAKEUP key to start! \r\n");

    /* initialize transmit message */
    transmit_message.rtr = 0U;
    transmit_message.ide = 0U;
    transmit_message.code = CAN_MB_TX_STATUS_DATA;
    transmit_message.brs = 0U;
    transmit_message.fdf = 0U;
    transmit_message.prio = 0U;
    transmit_message.data_bytes = BUFFER_SIZE;
    /* tx message content */
    transmit_message.data = (uint32_t *)(tx_data);
    transmit_message.id = 0x55U;

    receive_message.rtr = 0U;
    receive_message.ide = 0U;
    receive_message.code = CAN_MB_RX_STATUS_EMPTY;
    /* rx mailbox */
    receive_message.id = 0x55U;
    receive_message.data = (uint32_t *)(rx_data);
    can_mailbox_config(CAN1, 0U, &receive_message);

    while(1) {
        /* test whether the WAKEUP key is pressed */
        if(1U == gd_eval_key_state_get(KEY_WAKEUP)) {
            delay_ms(100U);
            if(1U == gd_eval_key_state_get(KEY_WAKEUP)) {
                /* transmit message */
                can_mailbox_config(CAN2, 1U, &transmit_message);

                printf("\r\nCAN2 transmit data: \r\n");
                for(i = 0U; i < transmit_message.data_bytes; i++) {
                    printf("%02x ", tx_data[i]);
                }
                /* waiting for the WAKEUP key up */
                while(1U == gd_eval_key_state_get(KEY_WAKEUP));
            }
        }
        communication_check();
    }
}

/*!
    \brief      configure GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
void can_gpio_config(void)
{
    /* configure CAN1 and CAN2 clock source */
    rcu_can_clock_config(IDX_CAN1, RCU_CANSRC_APB2);
    rcu_can_clock_config(IDX_CAN2, RCU_CANSRC_APB2);
    /* enable CAN clock */
    rcu_periph_clock_enable(RCU_CAN1);
    rcu_periph_clock_enable(RCU_CAN2);
    /* enable CAN port clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOF);

    /* configure CAN1_RX GPIO */
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_5);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_af_set(GPIOB, GPIO_AF_9, GPIO_PIN_5);
    /* configure CAN1_TX GPIO */
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_13);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_13);
    gpio_af_set(GPIOB, GPIO_AF_9, GPIO_PIN_13);

    /* configure CAN2_RX GPIO */
    gpio_output_options_set(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_6);
    gpio_mode_set(GPIOF, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_af_set(GPIOF, GPIO_AF_2, GPIO_PIN_6);
    /* configure CAN2_TX GPIO */
    gpio_output_options_set(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_7);
    gpio_mode_set(GPIOF, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_7);
    gpio_af_set(GPIOF, GPIO_AF_2, GPIO_PIN_7);
}

/*!
    \brief      configure BSP
    \param[in]  none
    \param[out] none
    \retval     none
*/
void bsp_board_config(void)
{
    /* configure USART */
    gd_eval_com_init(EVAL_COM);

    /* configure WAKEUP key */
    gd_eval_key_init(KEY_WAKEUP, KEY_MODE_GPIO);

    /* configure LED2 */
    gd_eval_led_init(LED2);
    gd_eval_led_off(LED2);
}

/*!
    \brief      initialize CAN function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void can_config(void)
{
    can_parameter_struct can_parameter;

    /* initialize CAN register */
    can_deinit(CAN1);
    can_deinit(CAN2);
    /* initialize CAN */
    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);

    /* initialize CAN parameters */
    can_parameter.internal_counter_source = CAN_TIMER_SOURCE_BIT_CLOCK;
    can_parameter.self_reception = DISABLE;
    can_parameter.mb_tx_order = CAN_TX_HIGH_PRIORITY_MB_FIRST;
    can_parameter.mb_tx_abort_enable = ENABLE;
    can_parameter.local_priority_enable = DISABLE;
    can_parameter.mb_rx_ide_rtr_type = CAN_IDE_RTR_FILTERED;
    can_parameter.mb_remote_frame = CAN_STORE_REMOTE_REQUEST_FRAME;
    can_parameter.rx_private_filter_queue_enable = DISABLE;
    can_parameter.edge_filter_enable = DISABLE;
    can_parameter.protocol_exception_enable = DISABLE;
    can_parameter.rx_filter_order = CAN_RX_FILTER_ORDER_MAILBOX_FIRST;
    can_parameter.memory_size = CAN_MEMSIZE_32_UNIT;
    /* filter configuration */
    can_parameter.mb_public_filter = 0U;
    /* baud rate 1Mbps, sample point at 80% */
    can_parameter.resync_jump_width = 1U;
    can_parameter.prop_time_segment = 2U;
    can_parameter.time_segment_1 = 5U;
    can_parameter.time_segment_2 = 2U;
    can_parameter.prescaler = 30U;

    /* initialize CAN */
    can_init(CAN1, &can_parameter);
    can_init(CAN2, &can_parameter);

    /* configure CAN1 NVIC */
    nvic_irq_enable(CAN1_Message_IRQn, 0U, 0U);

    /* enable CAN MB0 interrupt */
    can_interrupt_enable(CAN1, CAN_INT_MB0);

    can_operation_mode_enter(CAN2, CAN_NORMAL_MODE);
    can_operation_mode_enter(CAN1, CAN_NORMAL_MODE);
}

/*!
    \brief      check received data
    \param[in]  none
    \param[out] none
    \retval     none
*/
void communication_check(void)
{
    uint8_t i = 0U;
    /* CAN1 receive data correctly, the received data is printed */
    if(SET == can1_receive_flag) {
        can1_receive_flag = RESET;

        /* check the receive message */
        can_mailbox_receive_data_read(CAN1, 0U, &receive_message);
        if(0U == memcmp(rx_data, tx_data, receive_message.data_bytes)) {
            printf("\r\nCAN1 receive data: \r\n");
            for(i = 0U; i < receive_message.data_bytes; i++) {
                printf("%02x ", rx_data[i]);
            }
            gd_eval_led_toggle(LED2);
        }
    }
}

/*!
    \brief      enable the CPU Cache
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

/* retarget the C library printf function to the usart */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM, (uint8_t)ch);
    while(RESET == usart_flag_get(EVAL_COM, USART_FLAG_TBE));
    return ch;
}
