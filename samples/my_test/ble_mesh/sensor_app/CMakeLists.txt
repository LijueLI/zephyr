#SPDX-License-Identifier : Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
set(QEMU_EXTRA_FLAGS -s)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(sensor_app)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})