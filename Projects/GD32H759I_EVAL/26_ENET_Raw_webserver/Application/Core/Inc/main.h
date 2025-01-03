/*!
    \file    main.h
    \brief   the header file of main 
    
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

#ifndef MAIN_H
#define MAIN_H

#include "gd32h7xx.h"
#include "stdint.h"
#include "gd32h7xx_enet_eval.h"

//#define USE_DHCP    /* enable DHCP, if disabled static address is used */

//#define USE_ENET_INTERRUPT

/*The USE_ENET0 and USE_ENET1 macros cannot be opened at the same time*/
#define USE_ENET0
//#define USE_ENET1

//#define TIMEOUT_CHECK_USE_LWIP

/* MAC address: BOARD_MAC_ADDR0:BOARD_MAC_ADDR1:BOARD_MAC_ADDR2:BOARD_MAC_ADDR3:BOARD_MAC_ADDR4:BOARD_MAC_ADDR5 */
#define BOARD_MAC_ADDR0   2
#define BOARD_MAC_ADDR1   0xA
#define BOARD_MAC_ADDR2   0xF
#define BOARD_MAC_ADDR3   0xE
#define BOARD_MAC_ADDR4   0xD
#define BOARD_MAC_ADDR5   6
 
/* static IP address: BOARD_IP_ADDR0.BOARD_IP_ADDR1.BOARD_IP_ADDR2.BOARD_IP_ADDR3 */
#define BOARD_IP_ADDR0   10
#define BOARD_IP_ADDR1   50
#define BOARD_IP_ADDR2   3
#define BOARD_IP_ADDR3   210

/* net mask */
#define BOARD_NETMASK_ADDR0   255
#define BOARD_NETMASK_ADDR1   255
#define BOARD_NETMASK_ADDR2   255
#define BOARD_NETMASK_ADDR3   0

/* gateway address */
#define BOARD_GW_ADDR0   10
#define BOARD_GW_ADDR1   50
#define BOARD_GW_ADDR2   3
#define BOARD_GW_ADDR3   1

/* MII and RMII mode selection */
#define RMII_MODE  // user have to provide the 50 MHz clock by soldering a 50 MHz oscillator
//#define MII_MODE

/* clock the PHY from external 25MHz crystal (only for MII mode) */
#ifdef  MII_MODE
#define PHY_CLOCK_MCO
#endif

/* function declarations */
/* updates the system local time */
void time_update(void);
/* insert a delay time */
void delay_10ms(uint32_t ncount);

#endif /* MAIN_H */
