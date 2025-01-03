project(GD32H757V_START LANGUAGES C CXX ASM)

add_library(GD32H757V_START OBJECT
    ${DRIVERS_DIR}/BSP/GD32H757V_START/gd32h757v_start.c
    )

target_include_directories(GD32H757V_START PUBLIC
    ${DRIVERS_DIR}/BSP/GD32H757V_START
    )

target_link_libraries(GD32H757V_START PUBLIC GD32H7xx_standard_peripheral)
