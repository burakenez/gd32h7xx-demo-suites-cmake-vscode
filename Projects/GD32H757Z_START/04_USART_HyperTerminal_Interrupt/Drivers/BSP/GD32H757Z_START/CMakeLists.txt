project(GD32H757Z_START LANGUAGES C CXX ASM)

add_library(GD32H757Z_START OBJECT
    ${DRIVERS_DIR}/BSP/GD32H757Z_START/gd32h757z_start.c
    )

target_include_directories(GD32H757Z_START PUBLIC
    ${DRIVERS_DIR}/BSP/GD32H757Z_START
    )

target_link_libraries(GD32H757Z_START PUBLIC GD32H7xx_standard_peripheral)
