project(GD32H7xx_standard_peripheral LANGUAGES C CXX ASM)

# Comment-out unused source files.
add_library(GD32H7xx_standard_peripheral OBJECT
	${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_adc.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_can.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_cau.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_cau_aes.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_cau_des.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_cau_tdes.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_cmp.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_cpdm.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_crc.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_ctc.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_dac.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_dbg.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_dci.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_dma.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_edout.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_efuse.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_enet.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_exmc.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_exti.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_fac.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_fmc.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_fwdgt.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_gpio.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_hau.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_hau_sha_md5.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_hpdf.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_hwsem.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_i2c.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_ipa.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_lpdts.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_mdio.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_mdma.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_misc.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_ospi.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_ospim.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_pmu.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_rameccmu.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_rcu.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_rspdif.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_rtc.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_rtdec.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_sai.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_sdio.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_spi.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_syscfg.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_timer.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_tli.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_tmu.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_trigsel.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_trng.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_usart.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_vref.c
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Source/gd32h7xx_wwdgt.c
    )

target_include_directories(GD32H7xx_standard_peripheral PUBLIC
    ${DRIVERS_DIR}/GD32H7xx_standard_peripheral/Include
    )

# CMSIS header only library is linked.
target_link_libraries(GD32H7xx_standard_peripheral PUBLIC CMSIS)
