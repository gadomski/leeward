cmake_minimum_required(VERSION 3.18)
project(Leeward)

find_package(PDAL 1.9 REQUIRED CONFIG)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_custom_target(leeward
    DEPENDS ../)

add_library(pdal_plugin_filter_leeward SHARED Leeward.cpp)
target_link_libraries(pdal_plugin_filter_leeward ${PDAL_LIBRARIES})