# Copyright (c) 2026 FoBE Studio
# SPDX-License-Identifier: Apache-2.0

# Used by LBM
zephyr_library_compile_definitions(LR11XX)
zephyr_library_compile_definitions(LR11XX_TRANSCEIVER)
zephyr_library_compile_definitions(LR11XX_DISABLE_WARNINGS)

# Allow modem options
set(ALLOW_CSMA_BUILD true)

set(LBM_LR11XX_LIB_DIR ${LBM_LIB_RADIO_DRIVERS_DIR}/lr11xx_driver/src)
zephyr_include_directories(${LBM_LR11XX_LIB_DIR})

#-----------------------------------------------------------------------------
# Radio specific sources
#-----------------------------------------------------------------------------
zephyr_library_sources(
  ${LBM_LR11XX_LIB_DIR}/lr11xx_bootloader.c
  ${LBM_LR11XX_LIB_DIR}/lr11xx_crypto_engine.c
  ${LBM_LR11XX_LIB_DIR}/lr11xx_driver_version.c
  ${LBM_LR11XX_LIB_DIR}/lr11xx_lr_fhss.c
  ${LBM_LR11XX_LIB_DIR}/lr11xx_radio.c
  ${LBM_LR11XX_LIB_DIR}/lr11xx_radio_timings.c
  ${LBM_LR11XX_LIB_DIR}/lr11xx_regmem.c
  ${LBM_LR11XX_LIB_DIR}/lr11xx_rttof.c
  ${LBM_LR11XX_LIB_DIR}/lr11xx_system.c
  ${LBM_LIB_SMTC_MODEM_CORE_DIR}/smtc_ral/src/ral_lr11xx.c
  ${LBM_LIB_SMTC_MODEM_CORE_DIR}/smtc_ralf/src/ralf_lr11xx.c
)
