cmake_minimum_required(VERSION 3.18)
project(Leeward)

find_package(PDAL 1.9 REQUIRED CONFIG)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_custom_target(leeward
    DEPENDS ../target/release/libleeward.a ${CMAKE_BINARY_DIR}/leeward.h)
add_custom_command(OUTPUT ../target/release/libleeward.a
    COMMAND cargo build --release
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../src/capi.rs)
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/leeward.h
    COMMAND cbindgen --output ${CMAKE_BINARY_DIR}/leeward.h
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../src/capi.rs)

add_library(pdal_plugin_filter_leeward SHARED LeewardFilter.cpp)
target_link_libraries(pdal_plugin_filter_leeward PRIVATE ${PDAL_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/../target/release/libleeward.a)
target_include_directories(pdal_plugin_filter_leeward PRIVATE ${CMAKE_BINARY_DIR})
add_dependencies(pdal_plugin_filter_leeward leeward)