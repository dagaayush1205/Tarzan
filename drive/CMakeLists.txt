cmake_minimum_required(VERSION 3.13.1)
set(DTC_OVERLAY_FILE "boards/app.overlay")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(Tarzan LANGUAGES C)

target_sources(app PRIVATE src/main.c)
 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wpedantic -Wall -Wextra")

execute_process(
    COMMAND git describe --always --broken --dirty --tags
    OUTPUT_VARIABLE TARZAN_GIT_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
add_definitions(-DTARZAN_GIT_VERSION=\"${TARZAN_GIT_VERSION}\")


