# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

if("HERO_BLE" IN_LIST MBED_TARGET_LABELS)
    add_subdirectory(TARGET_HERO_BLE)
endif()

target_include_directories(mbed-core
    INTERFACE
        .
)

