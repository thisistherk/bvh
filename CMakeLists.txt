#
# Main project CMake file
#
# SPDX-License-Identifier: MIT
# This code is licensed under MIT license (see LICENSE file for details)
#

cmake_minimum_required(VERSION 3.0)

project(bvh)

find_package(embree 3.0 QUIET)

if(EMBREE_LIBRARY)
    message(STATUS "Embree found at ${EMBREE_LIBRARY}")
else()
    message(STATUS "Embree not found")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(DIR_SOLIDS ${CMAKE_CURRENT_SOURCE_DIR}/solids)

add_subdirectory(bvh)
add_subdirectory(test)
