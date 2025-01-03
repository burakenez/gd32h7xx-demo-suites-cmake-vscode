project(GD32H757J_START LANGUAGES C CXX ASM)

add_library(GD32H757J_START OBJECT
    ${DRIVERS_DIR}/BSP/GD32H757J_START/gd32h757j_start.c
    )

target_include_directories(GD32H757J_START PUBLIC
    ${DRIVERS_DIR}/BSP/GD32H757J_START
    )

target_link_libraries(GD32H757J_START PUBLIC GD32H7xx_standard_peripheral)
