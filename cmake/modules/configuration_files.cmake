# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2021, Nordic Semiconductor ASA

# Zephyr build system configuration files.
#
# Locate the Kconfig and DT config files that are to be used.
# Also, locate the appropriate application config directory.
#
# Outcome:
# The following variables will be defined when this CMake module completes:
#
# - CONF_FILE:              List of Kconfig fragments
# - EXTRA_CONF_FILE:        List of additional Kconfig fragments
# - DTC_OVERLAY_FILE:       List of devicetree overlay files
# - EXTRA_DTC_OVERLAY_FILE  List of additional devicetree overlay files
# - DTS_EXTRA_CPPFLAGS      List of additional devicetree preprocessor defines
# - APPLICATION_CONFIG_DIR: Root folder for application configuration
#
# If any of the above variables are already set when this CMake module is
# loaded, then no changes to the variable will happen.
#
# Variables set by this module and not mentioned above are considered internal
# use only and may be removed, renamed, or re-purposed without prior notice.

include_guard(GLOBAL)

include(extensions)

# Merge in variables from other sources (e.g. sysbuild)
zephyr_get(FILE_SUFFIX SYSBUILD GLOBAL)

zephyr_get(APPLICATION_CONFIG_DIR SYSBUILD GLOBAL)
zephyr_file(APPLICATION_ROOT APPLICATION_CONFIG_DIR)
set_ifndef(APPLICATION_CONFIG_DIR ${APPLICATION_SOURCE_DIR})
string(CONFIGURE ${APPLICATION_CONFIG_DIR} APPLICATION_CONFIG_DIR)

# Re-run CMake when a Kconfig fragment or a devicetree overlay is added to, or
# removed from, any of the directories searched below. Nothing else watches them,
# so without this the new file is only noticed once something unrelated happens to
# trigger a reconfigure.
#
# Not in script mode: twister pre-filters test cases by running this module
# through package_helper.cmake with cmake -P, and CONFIGURE_DEPENDS is rejected
# outright there. There is no build system to re-run in that mode anyway.
if(NOT CMAKE_SCRIPT_MODE_FILE)
  file(GLOB configuration_files_watched CONFIGURE_DEPENDS
       ${APPLICATION_CONFIG_DIR}/*.conf
       ${APPLICATION_CONFIG_DIR}/*.overlay
       ${APPLICATION_CONFIG_DIR}/socs/*.conf
       ${APPLICATION_CONFIG_DIR}/socs/*.overlay
       ${APPLICATION_CONFIG_DIR}/boards/*.conf
       ${APPLICATION_CONFIG_DIR}/boards/*.overlay
  )
endif()

zephyr_get(CONF_FILE SYSBUILD LOCAL)

# CONF_FILE is written to the CMake cache further down, so on every later CMake run
# zephyr_get() hands back the list discovered the first time and the search below is
# skipped. That freezes the set of Kconfig fragments for the lifetime of the build
# directory: one added to socs/ or boards/ afterwards is ignored until a pristine
# build, and the firmware silently stops matching the configuration in the tree.
#
# Remember what was discovered in CACHED_CONF_FILE, and when the cache still holds
# exactly that, drop it and search again. A value that differs came from the user -
# -DCONF_FILE=, the environment, sysbuild or the application's CMakeLists.txt - and
# is left untouched.
if(DEFINED CONF_FILE AND DEFINED CACHED_CONF_FILE
   AND "${CONF_FILE}" STREQUAL "${CACHED_CONF_FILE}")
  unset(CONF_FILE)
  unset(CONF_FILE CACHE)
endif()

if(NOT DEFINED CONF_FILE)
  set(conf_file_discovered TRUE)
  zephyr_file(CONF_FILES ${APPLICATION_CONFIG_DIR} KCONF CONF_FILE NAMES "prj.conf" SUFFIX ${FILE_SUFFIX} REQUIRED)
  zephyr_file(CONF_FILES ${APPLICATION_CONFIG_DIR}/socs KCONF CONF_FILE QUALIFIERS SUFFIX ${FILE_SUFFIX})
  zephyr_file(CONF_FILES ${APPLICATION_CONFIG_DIR}/boards KCONF CONF_FILE SUFFIX ${FILE_SUFFIX})
else()
  string(CONFIGURE "${CONF_FILE}" CONF_FILE_EXPANDED)
  string(REPLACE " " ";" CONF_FILE_AS_LIST "${CONF_FILE_EXPANDED}")
  list(LENGTH CONF_FILE_AS_LIST CONF_FILE_LENGTH)
  if(${CONF_FILE_LENGTH} EQUAL 1)
    get_filename_component(CONF_FILE_NAME ${CONF_FILE} NAME)
    if(${CONF_FILE_NAME} MATCHES "prj_(.*).conf")
      set(CONF_FILE_FORCE_CACHE FORCE)
    endif()
  endif()
endif()

set(APPLICATION_CONFIG_DIR ${APPLICATION_CONFIG_DIR} CACHE PATH "The application configuration folder" FORCE)
set(CONF_FILE ${CONF_FILE} CACHE STRING "If desired, you can build the application using\
the configuration settings specified in an alternate .conf file using this parameter. \
These settings will override the settings in the application’s .config file or its default .conf file.\
Multiple files may be listed, e.g. CONF_FILE=\"prj1.conf;prj2.conf\" \
The CACHED_CONF_FILE is internal Zephyr variable used between CMake runs. \
To change CONF_FILE, use the CONF_FILE variable." ${CONF_FILE_FORCE_CACHE})

if(conf_file_discovered)
  set(CACHED_CONF_FILE ${CONF_FILE} CACHE INTERNAL "Kconfig fragments found by Zephyr")
else()
  unset(CACHED_CONF_FILE CACHE)
endif()

# The CONF_FILE variable is now set to its final value.
zephyr_boilerplate_watch(CONF_FILE)

zephyr_get(DTC_OVERLAY_FILE SYSBUILD LOCAL)

# Same as for CONF_FILE above: forget a list we discovered ourselves, so that an
# overlay added to socs/ or boards/ is picked up without a pristine build.
if(DEFINED DTC_OVERLAY_FILE AND DEFINED CACHED_DTC_OVERLAY_FILE
   AND "${DTC_OVERLAY_FILE}" STREQUAL "${CACHED_DTC_OVERLAY_FILE}")
  unset(DTC_OVERLAY_FILE)
  unset(DTC_OVERLAY_FILE CACHE)
endif()

if(NOT DEFINED DTC_OVERLAY_FILE)
  set(dtc_overlay_file_discovered TRUE)
endif()

# If DTC_OVERLAY_FILE is not set by the user, look for SoC and board-specific overlays
# in the 'boards' and `soc` configuration subdirectories.
if(NOT DEFINED DTC_OVERLAY_FILE)
  zephyr_file(CONF_FILES ${APPLICATION_CONFIG_DIR}/socs DTS DTC_OVERLAY_FILE QUALIFIERS SUFFIX ${FILE_SUFFIX})
  zephyr_file(CONF_FILES ${APPLICATION_CONFIG_DIR}/boards DTS DTC_OVERLAY_FILE SUFFIX ${FILE_SUFFIX})
endif()

# If still not found, search for other overlays in the configuration directory.
if(NOT DEFINED DTC_OVERLAY_FILE)
  zephyr_file(CONF_FILES ${APPLICATION_CONFIG_DIR} DTS DTC_OVERLAY_FILE)

  if(NOT DEFINED DTC_OVERLAY_FILE)
    zephyr_file(CONF_FILES ${APPLICATION_CONFIG_DIR} DTS DTC_OVERLAY_FILE
                NAMES "app.overlay" SUFFIX ${FILE_SUFFIX}
    )
  endif()
endif()

set(DTC_OVERLAY_FILE ${DTC_OVERLAY_FILE} CACHE STRING "If desired, you can \
build the application using the DT configuration settings specified in an \
alternate .overlay file using this parameter. These settings will override the \
settings in the board's .dts file. Multiple files may be listed, e.g. \
DTC_OVERLAY_FILE=\"dts1.overlay dts2.overlay\"")

if(dtc_overlay_file_discovered)
  set(CACHED_DTC_OVERLAY_FILE ${DTC_OVERLAY_FILE} CACHE INTERNAL
      "Devicetree overlays found by Zephyr")
else()
  unset(CACHED_DTC_OVERLAY_FILE CACHE)
endif()

# The DTC_OVERLAY_FILE variable is now set to its final value.
zephyr_boilerplate_watch(DTC_OVERLAY_FILE)

zephyr_get(EXTRA_CONF_FILE SYSBUILD LOCAL VAR EXTRA_CONF_FILE OVERLAY_CONFIG MERGE REVERSE)
zephyr_get(EXTRA_DTC_OVERLAY_FILE SYSBUILD LOCAL MERGE REVERSE)
zephyr_get(DTS_EXTRA_CPPFLAGS SYSBUILD LOCAL MERGE REVERSE)
build_info(application source-dir VALUE ${APPLICATION_SOURCE_DIR})
build_info(application configuration-dir VALUE ${APPLICATION_CONFIG_DIR})
