cmake_minimum_required(VERSION 3.6)
project(moxie-vm)
add_definitions(-Wall -Werror -Wpedantic)

include_directories(/usr/lib/modules/${CMAKE_SYSTEM_VERSION}/build/include)

set(SOURCE_FILES ${CMAKE_SOURCE_DIR}/src/main.c src/mox/processor.c)
add_executable(moxie-vm ${SOURCE_FILES})
