cmake_minimum_required(VERSION 3.13.1)
set(DTC_OVERLAY_FILE "boards/app.overlay")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(Tarzan LANGUAGES C)

target_sources(app PRIVATE src/main.c)
 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wpedantic -Wall -Wextra")

execute_process(
    COMMAND git describe
    OUTPUT_VARIABLE GIT_BRANCH_NAME
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
add_definitions(-DGIT_BRANCH_NAME=\"${GIT_COMMIT_ID}\")


