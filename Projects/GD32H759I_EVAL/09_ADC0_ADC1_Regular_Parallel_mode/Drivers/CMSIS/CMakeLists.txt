project(CMSIS LANGUAGES C CXX ASM)

add_library(CMSIS INTERFACE)

target_include_directories(CMSIS INTERFACE
    ${DRIVERS_DIR}/CMSIS/
    ${DRIVERS_DIR}/CMSIS/GD/GD32H7xx/Include

	# Added directory of "gd32h7xx_libopt.h".
    ${CMAKE_SOURCE_DIR}/Application/Core/Inc
    )
