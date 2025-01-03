project(lwip LANGUAGES C CXX ASM)

add_library(lwip OBJECT
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/api/api_lib.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/api/api_msg.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/api/err.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/api/if_api.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/api/netbuf.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/api/netdb.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/api/netifapi.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/api/sockets.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/api/tcpip.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/altcp.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/altcp_alloc.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/altcp_tcp.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/def.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/dns.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/inet_chksum.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/init.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/ip.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/ipv4/autoip.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/ipv4/dhcp.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/ipv4/etharp.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/ipv4/icmp.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/ipv4/igmp.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/ipv4/ip4.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/ipv4/ip4_addr.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/ipv4/ip4_frag.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/mem.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/memp.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/netif.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/pbuf.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/raw.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/stats.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/sys.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/tcp.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/tcp_in.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/tcp_out.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/timeouts.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/core/udp.c
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/netif/ethernet.c
    )

target_include_directories(lwip PUBLIC
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/include
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/include/ipv4
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/include/netif
    ${MIDDLEWARES_DIR}/Third_Party/lwip/src/include/lwip
    ${CMAKE_SOURCE_DIR}/Application/Core/Inc
    ${CMAKE_SOURCE_DIR}/Application/lwip/port/GD32H7xx
    ${CMAKE_SOURCE_DIR}/Application/lwip/port/GD32H7xx/FreeRTOS
    ${CMAKE_SOURCE_DIR}/Application/lwip/port/GD32H7xx/arch
    )

