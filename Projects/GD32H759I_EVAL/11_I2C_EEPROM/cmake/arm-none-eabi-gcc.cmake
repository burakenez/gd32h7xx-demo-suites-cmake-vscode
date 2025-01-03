# You can change TOOLCHAIN_DIRECTORY if you want to use different toolchain.
set(TOOLCHAIN_DIRECTORY "${CMAKE_SOURCE_DIR}/../../../Tools/xpack-arm-none-eabi-gcc-11.3.1-1.1/bin")

set(CMAKE_C_FLAGS_DEBUG "")
set(CMAKE_CXX_FLAGS_DEBUG "")
set(CMAKE_ASM_FLAGS_DEBUG "")
set(CMAKE_C_FLAGS_RELEASE "")
set(CMAKE_CXX_FLAGS_RELEASE "")
set(CMAKE_ASM_FLAGS_RELEASE "")

set(CMAKE_COLOR_DIAGNOSTICS ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

if(WIN32)
    set(TOOLCHAIN_SUFFIX ".exe")
endif()

set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(TOOLCHAIN_PREFIX                "arm-none-eabi-")
if(DEFINED TOOLCHAIN_DIRECTORY)
    set(TOOLCHAIN_PREFIX            "${TOOLCHAIN_DIRECTORY}/${TOOLCHAIN_PREFIX}")
endif()

set(FLAGS                           "-std=gnu11 -fstack-usage -fdata-sections -ffunction-sections -fmessage-length=0 -fsigned-char -mthumb -Wall -Wno-missing-braces -Wno-format -Wno-strict-aliasing -Wl,--gc-sections")
set(ASM_FLAGS                       "-x assembler-with-cpp")
set(CPP_FLAGS                       "-fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc${TOOLCHAIN_SUFFIX} ${FLAGS})
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER} ${ASM_FLAGS})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++${TOOLCHAIN_SUFFIX} ${FLAGS} ${CPP_FLAGS})
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy${TOOLCHAIN_SUFFIX})
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size${TOOLCHAIN_SUFFIX})
set(CMAKE_OBJDUMP                   ${TOOLCHAIN_PREFIX}objdump${TOOLCHAIN_SUFFIX})
set(CMAKE_AS                        ${TOOLCHAIN_PREFIX}as${TOOLCHAIN_SUFFIX})
set(CMAKE_LD                        ${TOOLCHAIN_PREFIX}ld${TOOLCHAIN_SUFFIX})

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
