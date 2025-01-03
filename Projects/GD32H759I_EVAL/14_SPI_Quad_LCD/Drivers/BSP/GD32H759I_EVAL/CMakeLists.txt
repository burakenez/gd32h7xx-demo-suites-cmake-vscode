project(GD32H759I_EVAL LANGUAGES C CXX ASM)

add_library(GD32H759I_EVAL OBJECT
    ${DRIVERS_DIR}/BSP/GD32H759I_EVAL/gd32h759i_eval.c
    )

target_include_directories(GD32H759I_EVAL PUBLIC
    ${DRIVERS_DIR}/BSP/GD32H759I_EVAL
    )

target_link_libraries(GD32H759I_EVAL PUBLIC GD32H7xx_standard_peripheral)
