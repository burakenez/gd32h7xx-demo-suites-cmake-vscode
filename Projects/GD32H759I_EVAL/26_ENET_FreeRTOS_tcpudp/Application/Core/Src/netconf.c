/*!
    \file    netconf.c
    \brief   network connection configuration 

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

#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/dhcp.h"
#include "ethernetif.h"
#include "main.h"
#include "netconf.h"
#include "tcpip.h"
#include <stdio.h>
#include "lwip/errno.h"
#include "queue.h"

#define DHCP_TRIES_MAX_TIMES       3

extern int errno;

typedef enum 
{ 
    DHCP_ADDR_NONE = 0,
    DHCP_ADDR_BEGIN,
    DHCP_ADDR_GOT,
    DHCP_ADDR_FAIL
}dhcp_addr_status_enum;

#ifdef USE_DHCP
dhcp_addr_status_enum dhcp_state = DHCP_ADDR_NONE;
#endif /* USE_DHCP */

struct netif g_mynetif0, g_mynetif1;
ip_addr_t ip_address = {0};

/*!
    \brief      initializes the LwIP stack
    \param[in]  none
    \param[out] none
    \retval     none
*/
void lwip_stack_init(void)
{
    ip_addr_t gd_ipaddr;
    ip_addr_t gd_netmask;
    ip_addr_t gd_gw;

    /* create tcp_ip stack thread */
    tcpip_init( NULL, NULL );

    /* IP address setting */
#ifdef USE_DHCP
    gd_ipaddr.addr = 0;
    gd_netmask.addr = 0;
    gd_gw.addr = 0;
#else
    IP4_ADDR(&gd_ipaddr, BOARD_IP_ADDR0, BOARD_IP_ADDR1, BOARD_IP_ADDR2, BOARD_IP_ADDR3);
    IP4_ADDR(&gd_netmask, BOARD_NETMASK_ADDR0, BOARD_NETMASK_ADDR1, BOARD_NETMASK_ADDR2, BOARD_NETMASK_ADDR3);
    IP4_ADDR(&gd_gw, BOARD_GW_ADDR0, BOARD_GW_ADDR1, BOARD_GW_ADDR2, BOARD_GW_ADDR3);

#endif /* USE_DHCP */

#ifdef USE_ENET0
    /* add a new network interface */
    netif_add(&g_mynetif0, &gd_ipaddr, &gd_netmask, &gd_gw, NULL, &ethernetif_init, &tcpip_input);

    /* set a default network interface */
    netif_set_default(&g_mynetif0);

    /* set a callback when interface is up/down */
    netif_set_status_callback(&g_mynetif0, lwip_netif_status_callback);

    /* set the flag of netif as NETIF_FLAG_LINK_UP */
    netif_set_link_up(&g_mynetif0);

    /* bring an interface up and set the flag of netif as NETIF_FLAG_UP */
    netif_set_up(&g_mynetif0);
#endif /* USE_ENET0 */

#ifdef USE_ENET1
    /* add a new network interface */
    netif_add(&g_mynetif1, &gd_ipaddr, &gd_netmask, &gd_gw, NULL, &ethernetif_init, &tcpip_input);

    /* set a default network interface */
    netif_set_default(&g_mynetif1);

    /* set a callback when interface is up/down */
    netif_set_status_callback(&g_mynetif1, lwip_netif_status_callback);

    /* set the flag of netif as NETIF_FLAG_LINK_UP */
    netif_set_link_up(&g_mynetif1);

    /* bring an interface up and set the flag of netif as NETIF_FLAG_UP */
    netif_set_up(&g_mynetif1);
#endif /* USE_ENET1 */
}

#ifdef USE_DHCP
/*!
    \brief      dhcp_task
    \param[in]  none
    \param[out] none
    \retval     none
*/
void dhcp_task(void * pvParameters)
{
    ip_addr_t gd_ipaddr;
    ip_addr_t gd_netmask;
    ip_addr_t gd_gw;

#ifdef USE_ENET0
    struct dhcp *dhcp_client0;
#endif /* USE_ENET0 */
#ifdef USE_ENET1
    struct dhcp *dhcp_client1;
#endif /* USE_ENET1 */
  
    for(;;){
#ifdef USE_ENET0
        switch(dhcp_state){
        case DHCP_ADDR_NONE:
            dhcp_start(&g_mynetif0);
            /* IP address should be set to 0 every time we want to assign a new DHCP address*/
            ip_address.addr = 0;
            dhcp_state = DHCP_ADDR_BEGIN;
            break;

        case DHCP_ADDR_BEGIN:    
            /* got the IP address */
            ip_address.addr = g_mynetif0.ip_addr.addr;

            if(0 != ip_address.addr){ 
                dhcp_state = DHCP_ADDR_GOT;
                printf("\r\nDHCP -- eval board ip address: %d.%d.%d.%d \r\n", ip4_addr1_16(&ip_address), \
                        ip4_addr2_16(&ip_address), ip4_addr3_16(&ip_address), ip4_addr4_16(&ip_address));
            }else{
                /* DHCP timeout */
                dhcp_client0 = netif_dhcp_data(&g_mynetif0);
                if(dhcp_client0->tries > DHCP_TRIES_MAX_TIMES){
                    dhcp_state = DHCP_ADDR_FAIL;
                    /* stop DHCP */
                    dhcp_stop(&g_mynetif0);

                /* use static address as IP address */
                IP4_ADDR(&gd_ipaddr, BOARD_IP_ADDR0, BOARD_IP_ADDR1, BOARD_IP_ADDR2, BOARD_IP_ADDR3);
                IP4_ADDR(&gd_netmask, BOARD_NETMASK_ADDR0, BOARD_NETMASK_ADDR1, BOARD_NETMASK_ADDR2, BOARD_NETMASK_ADDR3);
                IP4_ADDR(&gd_gw, BOARD_GW_ADDR0, BOARD_GW_ADDR1, BOARD_GW_ADDR2, BOARD_GW_ADDR3);
                netif_set_addr(&g_mynetif0, &gd_ipaddr, &gd_netmask, &gd_gw);
                }
            }
            break;

        default:
            break;
        }
#endif /* USE_ENET0 */
#ifdef USE_ENET1
        switch(dhcp_state){
        case DHCP_ADDR_NONE:
            dhcp_start(&g_mynetif1);
            /* IP address should be set to 0 every time we want to assign a new DHCP address*/
            ip_address.addr = 0;
            dhcp_state = DHCP_ADDR_BEGIN;
            break;

        case DHCP_ADDR_BEGIN:    
            /* got the IP address */
            ip_address.addr = g_mynetif1.ip_addr.addr;

            if(0 != ip_address.addr){ 
                dhcp_state = DHCP_ADDR_GOT;
                printf("\r\nDHCP -- eval board ip address: %d.%d.%d.%d \r\n", ip4_addr1_16(&ip_address), \
                        ip4_addr2_16(&ip_address), ip4_addr3_16(&ip_address), ip4_addr4_16(&ip_address));
            }else{
                /* DHCP timeout */
                dhcp_client1 = netif_dhcp_data(&g_mynetif1);
                if(dhcp_client1->tries > DHCP_TRIES_MAX_TIMES){
                    dhcp_state = DHCP_ADDR_FAIL;
                    /* stop DHCP */
                    dhcp_stop(&g_mynetif0);

                /* use static address as IP address */
                IP4_ADDR(&gd_ipaddr, BOARD_IP_ADDR0, BOARD_IP_ADDR1, BOARD_IP_ADDR2, BOARD_IP_ADDR3);
                IP4_ADDR(&gd_netmask, BOARD_NETMASK_ADDR0, BOARD_NETMASK_ADDR1, BOARD_NETMASK_ADDR2, BOARD_NETMASK_ADDR3);
                IP4_ADDR(&gd_gw, BOARD_GW_ADDR0, BOARD_GW_ADDR1, BOARD_GW_ADDR2, BOARD_GW_ADDR3);
                netif_set_addr(&g_mynetif1, &gd_ipaddr, &gd_netmask, &gd_gw);
                }
            }
            break;

        default:
            break;
        }
#endif /* USE_ENET1 */
        /* wait 250 ms */
        vTaskDelay(250);
    }   
}
#endif /* USE_DHCP */
