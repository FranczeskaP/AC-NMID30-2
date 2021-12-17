include(FindPkgConfig)
cmake_minimum_required(VERSION 3.10)
project(AcTest VERSION 1.0)

add_executable(AcTest AcTest.c)
pkg_search_module(MODBUS REQUIRED libmodbus)

target_include_directories(AcTest
    PUBLIC
    ${MODBUS_INCLUDE_DIRS})

target_link_libraries(AcTest 
    m #math lib
    ${MODBUS_LIBRARIES})

