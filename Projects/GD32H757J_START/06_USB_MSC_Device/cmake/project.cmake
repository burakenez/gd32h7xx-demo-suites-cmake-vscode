function(project_add_target_properties TARGET_NAME)

target_compile_definitions(${TARGET_NAME} PRIVATE
    "$<$<CONFIG:Debug>:DEBUG>"
    "$<$<NOT:$<CONFIG:Debug>>:RELEASE>"
	USE_USB_FS
    USE_USBHS0
    GD32H7XX
	)

target_compile_options(${TARGET_NAME} PRIVATE
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:ASM>>:-g3>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:-g3>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:-g3>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:ASM>>:-g0>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:-g0>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:-g0>"

    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:-O0>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:-O0>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:-Os>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:-Os>"

    -mcpu=cortex-m7
    -mfpu=fpv5-d16
    -mfloat-abi=hard
	-mcpu=cortex-m7
    -mfpu=fpv5-d16
    -mfloat-abi=hard
    )

target_link_options(${TARGET_NAME} PRIVATE 
    -mcpu=cortex-m7
    -mfpu=fpv5-d16
    -mfloat-abi=hard
	-mcpu=cortex-m7
    -mfpu=fpv5-d16
    -mfloat-abi=hard
    -mthumb
    -u _printf_float
    -static
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,--gc-sections
    -Wl,--start-group -lc -lm -Wl,--end-group
    )

target_link_libraries(${TARGET_NAME} PRIVATE 
    m # To use C math library. -lm should be end of the linker script.
    )

endfunction()
