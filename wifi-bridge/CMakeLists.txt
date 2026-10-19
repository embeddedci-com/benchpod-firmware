# BenchPod Wi-Fi bridge: ESP32-WROOM-32E, Ethernet (LAN8742A, RMII) <-> Wi-Fi station, layer 2.
cmake_minimum_required(VERSION 3.16)

# The OTA path refuses images whose project name differs, so keep this name.
set(PROJECT_VER "0.1.0")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(benchpod-wifi-bridge)
