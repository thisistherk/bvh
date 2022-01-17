//
// Interface that all BVHs implement
//
// SPDX-License-Identifier: MIT
// This code is licensed under MIT license (see LICENSE file for details)
//

#pragma once

#include <cmath>
#include <cstdint>


namespace bvh
{
    struct Mesh
    {
        uint32_t        vertices;
        uint32_t        triangles;
        const float*    positions;
        const uint32_t* indices;
    };

    struct Ray
    {
        float origin[3];
        float min_t;
        float direction[3];
        float max_t;
    };

    struct Hit
    {
        uint32_t triangle;
        float    barycentric[2];
    };


    constexpr uint32_t TRIANGLE_INVALID = static_cast<uint32_t>(-1);

    constexpr uint32_t TRACE_COHERENT = 0x0001;
    constexpr uint32_t TRACE_SHADOW   = 0x0002;

    class Base
    {
    public:
        // Virtual dtor
        virtual ~Base() = default;

        // Build the BVH
        virtual void build(const Mesh* mesh) = 0;

        // Trace rays against the BVH
        virtual void trace(uint32_t rays, Ray* input, Hit* output, uint32_t flags) = 0;
    };
}
