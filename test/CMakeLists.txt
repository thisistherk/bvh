#
# BVH test app
#
# SPDX-License-Identifier: MIT
# This code is licensed under MIT license (see LICENSE file for details)
#

add_executable(test
    ambient.cpp
    ambient.h
    entry.cpp
    fast_obj.h
    image.cpp
    image.h
    log.h
    mesh.cpp
    mesh.h
    timer.h
    v3.h
)

target_compile_options(test PRIVATE
    $<$<CXX_COMPILER_ID:AppleClang, Clang, GNU>: -Wall -Wextra -pedantic>
)
target_compile_features(test PRIVATE cxx_std_20)
target_compile_definitions(test PRIVATE "DIR_SOLIDS=\"${DIR_SOLIDS}\"")
target_link_libraries(test PRIVATE bvh)
