# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_MSPM0G3506)
    zephyr_compile_definitions(-D__MSPM0G3506__)
elseif(CONFIG_SOC_MSPM0G3507)
    zephyr_compile_definitions(-D__MSPM0G3507__)
endif()
zephyr_sources(soc.c)
set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")
