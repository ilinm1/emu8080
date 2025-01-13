cmake_minimum_required(VERSION 3.5.0)
project(emu8080 VERSION 0.1.0 LANGUAGES C)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
add_executable(emu8080 src/main.c src/system.c src/periphery/console.c src/periphery/bdos.c)
