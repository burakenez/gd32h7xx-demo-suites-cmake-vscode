project(GD32H7xx_usbhs_library LANGUAGES C CXX ASM)

add_library(GD32H7xx_usbhs_library OBJECT
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/driver/Source/drv_usb_core.c
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/driver/Source/drv_usb_host.c
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/driver/Source/drv_usbh_int.c
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/host/class/hid/Source/usbh_hid_core.c
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/host/class/hid/Source/usbh_standard_hid.c
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/host/core/Source/usbh_core.c
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/host/core/Source/usbh_enum.c
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/host/core/Source/usbh_pipe.c
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/host/core/Source/usbh_transc.c
    )

target_include_directories(GD32H7xx_usbhs_library PUBLIC
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/driver/Include
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/host/core/Include
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/host/class/hid/Include
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/ustd/common
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/ustd/class/hid
    )

target_link_libraries(GD32H7xx_usbhs_library PUBLIC GD32H759I_EVAL)
