cmake_minimum_required(VERSION 3.18)
project(LeewardPdal)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(Leeward REQUIRED)
find_package(PDAL 1.9 REQUIRED CONFIG)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(pdal_plugin_filter_leeward SHARED LeewardFilter.cpp)
target_link_libraries(pdal_plugin_filter_leeward PRIVATE ${PDAL_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/../target/release/libleeward.a)
target_include_directories(pdal_plugin_filter_leeward PRIVATE ${CMAKE_BINARY_DIR})
add_dependencies(pdal_plugin_filter_leeward leeward)

install(TARGETS pdal_plugin_filter_leeward)