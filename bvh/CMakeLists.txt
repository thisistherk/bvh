#
# Header only library of BVH implementations
#
# SPDX-License-Identifier: MIT
# This code is licensed under MIT license (see LICENSE file for details)
#

add_library(bvh INTERFACE)

target_include_directories(bvh INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_compile_features(bvh INTERFACE cxx_std_20)

if(EMBREE_LIBRARY)
    target_compile_definitions(bvh INTERFACE "BVH_EMBREE")
    target_link_libraries(bvh INTERFACE embree)
endif()
