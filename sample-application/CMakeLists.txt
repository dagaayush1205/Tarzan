cmake_minimum_required(VERSION 3.13.1)
set(BOARD nucleo_h7a3zi_q)
set(DTC_OVERLAY_FILE "boards/pwm_motor_fragment.overlay")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(kyvernitis LANGUAGES C)

target_sources(app PRIVATE src/main.c)

