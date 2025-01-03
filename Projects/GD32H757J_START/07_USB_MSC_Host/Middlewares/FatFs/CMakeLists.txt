project(FatFs LANGUAGES C CXX ASM)

add_library(FatFs OBJECT
    ${MIDDLEWARES_DIR}/Third_Party/FatFs/Source/ff.c
    ${MIDDLEWARES_DIR}/Third_Party/FatFs/Source/ffunicode.c
    )

target_include_directories(FatFs PUBLIC
    ${MIDDLEWARES_DIR}/Third_Party/FatFs/Source
    ${CMAKE_SOURCE_DIR}/Application/Core/Inc
    )

