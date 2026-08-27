# Copyright (c) 2026 FoBE Studio
# SPDX-License-Identifier: Apache-2.0

# Used by LBM
zephyr_library_compile_definitions(LR20XX)
zephyr_library_compile_definitions(LR2021)

# Allow modem options
set(ALLOW_CSMA_BUILD true)

set(LBM_LR20XX_LIB_DIR ${LBM_LIB_RADIO_DRIVERS_DIR}/lr20xx_driver)
zephyr_include_directories(${LBM_LR20XX_LIB_DIR}/inc)
zephyr_include_directories(${LORA_BASICS_MODEM_DIR}/lbm_examples/radio_hal)

#-----------------------------------------------------------------------------
# Radio specific sources
#-----------------------------------------------------------------------------
zephyr_library_sources(
  ${LBM_LR20XX_LIB_DIR}/src/lr20xx_driver_version.c
  ${LBM_LR20XX_LIB_DIR}/src/lr20xx_radio_common.c
  ${LBM_LR20XX_LIB_DIR}/src/lr20xx_radio_fifo.c
  ${LBM_LR20XX_LIB_DIR}/src/lr20xx_regmem.c
  ${LBM_LR20XX_LIB_DIR}/src/lr20xx_system.c
  ${LBM_LR20XX_LIB_DIR}/src/lr20xx_workarounds.c
  ${LBM_LR20XX_LIB_DIR}/src/lr20xx_radio_lora.c
  ${LBM_LR20XX_LIB_DIR}/src/lr20xx_radio_fsk.c
  ${LBM_LR20XX_LIB_DIR}/src/lr20xx_radio_flrc.c
  ${LBM_LR20XX_LIB_DIR}/src/lr20xx_radio_lr_fhss.c
  ${LBM_LR20XX_LIB_DIR}/src/lr20xx_rttof.c
  ${LBM_LIB_SMTC_MODEM_CORE_DIR}/smtc_ral/src/ral_lr20xx.c
  ${LBM_LIB_SMTC_MODEM_CORE_DIR}/smtc_ralf/src/ralf_lr20xx.c
)
