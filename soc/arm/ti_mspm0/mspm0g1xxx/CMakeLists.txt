# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_MSPM0G1106)
    zephyr_compile_definitions(-D__MSPM0G1106__)
endif()
zephyr_sources(soc.c)
