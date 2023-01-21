#
# Copyright (c) 2021 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE ZDebug)
endif()
# Point to the build type and common config files
set(CONF_FILE "prj.${CMAKE_BUILD_TYPE}.conf" "prj.common.conf")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(ultra_portable_vital_monitoring)

# NORDIC SDK APP START
add_subdirectory(source)
# NORDIC SDK APP END



