project(GD32H7xx_usbhs_library LANGUAGES C CXX ASM)

add_library(GD32H7xx_usbhs_library OBJECT
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/driver/Source/drv_usb_core.c
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/driver/Source/drv_usb_host.c
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/driver/Source/drv_usbh_int.c
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/host/class/msc/Source/usbh_msc_bbb.c
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/host/class/msc/Source/usbh_msc_core.c
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/host/class/msc/Source/usbh_msc_fatfs.c
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/host/class/msc/Source/usbh_msc_scsi.c
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/host/core/Source/usbh_core.c
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/host/core/Source/usbh_enum.c
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/host/core/Source/usbh_pipe.c
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/host/core/Source/usbh_transc.c
    )

target_include_directories(GD32H7xx_usbhs_library PUBLIC
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/driver/Include
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/host/core/Include
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/host/class/msc/Include
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/ustd/common
    ${DRIVERS_DIR}/GD32H7xx_usbhs_library/ustd/class/msc
    )

target_link_libraries(GD32H7xx_usbhs_library PUBLIC GD32H757J_START)
target_link_libraries(GD32H7xx_usbhs_library PUBLIC FatFs)
