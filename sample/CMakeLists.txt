cmake_minimum_required(VERSION 3.13.1)
set(DTC_OVERLAY_FILE "boards/app.overlay")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(Tarzan LANGUAGES C)

target_sources(app PRIVATE src/sbus_test.c)

