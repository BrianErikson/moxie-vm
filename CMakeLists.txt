cmake_minimum_required(VERSION 3.6)
project(moxie-vm)

include_directories(/usr/lib/modules/${CMAKE_SYSTEM_VERSION}/build/include)

set(SOURCE_FILES ${CMAKE_SOURCE_DIR}/src/main.c)
add_executable(moxie-vm ${SOURCE_FILES})
