# BenchPod Wi-Fi bridge: ESP32-WROOM-32E, Ethernet (LAN8742A, RMII) <-> Wi-Fi station, layer 2.
cmake_minimum_required(VERSION 3.16)

# The OTA path refuses images whose project name differs, so keep this name.
set(PROJECT_VER "0.1.1")
# The EMC test build (./build.sh emc) reports "<version>-emc": the pod does not auto-update a
# version it cannot parse, and HELLO and `version` show which image runs.
if(BRIDGE_EMC_BUILD)
    set(PROJECT_VER "${PROJECT_VER}-emc")
endif()
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(benchpod-wifi-bridge)
