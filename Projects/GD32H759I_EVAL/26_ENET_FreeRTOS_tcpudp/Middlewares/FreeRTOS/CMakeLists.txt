project(FreeRTOS LANGUAGES C CXX ASM)

add_library(FreeRTOS OBJECT
    ${MIDDLEWARES_DIR}/Third_Party/FreeRTOS/Source/croutine.c
    ${MIDDLEWARES_DIR}/Third_Party/FreeRTOS/Source/event_groups.c
    ${MIDDLEWARES_DIR}/Third_Party/FreeRTOS/Source/list.c
    ${MIDDLEWARES_DIR}/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
    ${MIDDLEWARES_DIR}/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    ${MIDDLEWARES_DIR}/Third_Party/FreeRTOS/Source/queue.c
    ${MIDDLEWARES_DIR}/Third_Party/FreeRTOS/Source/stream_buffer.c
    ${MIDDLEWARES_DIR}/Third_Party/FreeRTOS/Source/tasks.c
    ${MIDDLEWARES_DIR}/Third_Party/FreeRTOS/Source/timers.c
    )

target_include_directories(FreeRTOS PUBLIC
    ${MIDDLEWARES_DIR}/Third_Party/FreeRTOS/Source/include
    ${MIDDLEWARES_DIR}/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ${CMAKE_SOURCE_DIR}/Application/Core/Inc
    )

