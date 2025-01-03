project(GD32H7xx_usbhs_library LANGUAGES C CXX ASM)

add_library(GD32H7xx_usbhs_library OBJECT
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/device/class/cdc/Source/cdc_acm_core.c
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/device/core/Source/usbd_core.c
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/device/core/Source/usbd_enum.c
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/device/core/Source/usbd_transc.c
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/driver/Source/drv_usb_core.c
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/driver/Source/drv_usb_dev.c
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/driver/Source/drv_usbd_int.c
    )

target_include_directories(GD32H7xx_usbhs_library PUBLIC
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/driver/Include
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/device/core/Include
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/device/class/cdc/Include
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/ustd/common
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/ustd/class/cdc
    )

target_link_libraries(GD32H7xx_usbhs_library PUBLIC GD32H759I_EVAL)
