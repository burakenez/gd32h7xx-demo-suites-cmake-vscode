project(Application LANGUAGES C CXX ASM)

add_executable(Application)

set(TARGET_SRC
	# Core
    Core/Src/gd32h7xx_enet_eval.c
    Core/Src/gd32h7xx_it.c
    Core/Src/hello_gigadevice.c
    Core/Src/main.c
    Core/Src/netconf.c
    Core/Src/tcp_client.c
    Core/Src/udp_echo.c
    Core/Src/system_gd32h7xx.c	
	
    # lwip/port/GD32H7xx/Basic
    lwip/port/GD32H7xx/Basic/ethernetif.c

    # Startup
    Startup/startup_gd32h7xx.s

    # User
    User/syscalls.c
    )

target_sources(Application PRIVATE ${TARGET_SRC})

set(TARGET_INC_DIR
	${CMAKE_SOURCE_DIR}/Application/Core/Inc
    ${CMAKE_SOURCE_DIR}/Application/lwip/port/GD32H7xx
    ${CMAKE_SOURCE_DIR}/Application/lwip/port/GD32H7xx/Basic
    )

target_include_directories(Application PRIVATE ${TARGET_INC_DIR})

target_link_options(Application PRIVATE
	-T${CMAKE_SOURCE_DIR}/gd32h7xx_flash.ld -Xlinker
    -L${CMAKE_SOURCE_DIR}
	)

target_link_options(Application PRIVATE
	-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/$<TARGET_NAME:Application>.map
	)

target_link_libraries(Application PRIVATE CMSIS)
target_link_libraries(Application PRIVATE GD32H759I_EVAL)
target_link_libraries(Application PRIVATE GD32H7xx_standard_peripheral)
target_link_libraries(Application PRIVATE lwip)

add_custom_command(TARGET Application
    POST_BUILD
    COMMAND echo -- Running Post Build Commands
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:Application> ${CMAKE_CURRENT_BINARY_DIR}/$<TARGET_NAME:Application>.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:Application> ${CMAKE_CURRENT_BINARY_DIR}/$<TARGET_NAME:Application>.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:Application>
    COMMAND ${CMAKE_OBJDUMP} -h -S $<TARGET_FILE:Application> > ${CMAKE_CURRENT_BINARY_DIR}/$<TARGET_NAME:Application>.list
    COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:Application> > ${CMAKE_CURRENT_BINARY_DIR}/$<TARGET_NAME:Application>.bsz
    COMMAND ${CMAKE_SIZE} --format=sysv -x $<TARGET_FILE:Application> > ${CMAKE_CURRENT_BINARY_DIR}/$<TARGET_NAME:Application>.ssz
    )
