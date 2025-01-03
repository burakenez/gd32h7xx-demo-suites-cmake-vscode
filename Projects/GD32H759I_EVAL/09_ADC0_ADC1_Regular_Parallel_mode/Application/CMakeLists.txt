project(Application LANGUAGES C CXX ASM)

add_executable(Application)

set(TARGET_SRC
	# Core
    Core/Src/gd32h7xx_it.c
    Core/Src/main.c
    Core/Src/systick.c
    Core/Src/system_gd32h7xx.c	
	
    # Startup
    Startup/startup_gd32h7xx.s

    # User
    User/syscalls.c
    )

target_sources(Application PRIVATE ${TARGET_SRC})

set(TARGET_INC_DIR
	${CMAKE_SOURCE_DIR}/Application/Core/Inc
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
