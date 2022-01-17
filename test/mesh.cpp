//
// Triangle mesh data
//
// SPDX-License-Identifier: MIT
// This code is licensed under MIT license (see LICENSE file for details)
//

#include "mesh.h"

#define FAST_OBJ_IMPLEMENTATION
#include "fast_obj.h"

#include <algorithm>
#include <cassert>
#include <cmath>


// Read an OBJ file

Mesh::Mesh(const char* path)
{
    // Read OBJ file
    auto obj = fast_obj_read(path);
    if (!obj)
        return;


    // Mesh bounds
    _min = v3(+INFINITY, +INFINITY, +INFINITY);
    _max = v3(-INFINITY, -INFINITY, -INFINITY);


    // Copy positions.  Skip first - it's just a dummy value
    uint32_t n = obj->position_count - 1;
    _positions.resize(3 * n);
    const float* ps = obj->positions + 3;

    for (uint32_t ii = 0; ii < n; ii++)
    {
        v3 p(ps + 3 * ii);

        _min = min(_min, p);
        _max = max(_max, p);

        _positions[ii] = p;
    }


    // Copy indices.  Guess at them all being triangles for initial alloc.
    _indices.reserve(obj->face_count * 3);
    const auto* is = obj->indices;

    for (uint32_t ii = 0; ii < obj->face_count; ii++)
    {
        uint32_t vs = obj->face_vertices[ii];

        uint32_t i0 = (*is++).p - 1;
        uint32_t i1 = (*is++).p - 1;

        // Assume convex polygons so we can triangulate anything as a fan
        for (uint32_t jj = 2; jj < vs; jj++)
        {
            uint32_t i2 = (*is++).p - 1;

            assert(i0 < n);
            assert(i1 < n);
            assert(i2 < n);

            add_triangle(i0, i1, i2);
            i1 = i2;
        }
    }

    fast_obj_destroy(obj);
}


// Return centre of bounds

v3 Mesh::centre() const
{
    return 0.5f * (_min + _max);
}


// Return radius of bounding sphere

float Mesh::radius() const
{
    v3 delta = _max - _min;
    return len(0.5f * delta);
}


// Add a triangle

void Mesh::add_triangle(uint32_t a, uint32_t b, uint32_t c)
{
    _indices.push_back(a);
    _indices.push_back(b);
    _indices.push_back(c);
}


// Add a ground plane on the given axis

void Mesh::add_plane(uint32_t axis, float size)
{
    uint32_t x = (axis + 1) % 3;
    uint32_t y = (axis + 2) % 3;
    uint32_t z = axis;

    v3 delta = _max - _min;
    v3 dx(0.0f, 0.0f, 0.0f);
    v3 dy(0.0f, 0.0f, 0.0f);

    dx[x] = 0.5f * size * delta[x];
    dy[y] = 0.5f * size * delta[y];


    uint32_t first = vertices();
    _positions.resize(3 * (first + 4));

    v3 c = centre();
    c[z] = _min[z];

    _positions[first + 0] = c - dx - dy;
    _positions[first + 1] = c + dx - dy;
    _positions[first + 2] = c + dx + dy;
    _positions[first + 3] = c - dx + dy;

    add_triangle(first + 0, first + 1, first + 2);
    add_triangle(first + 0, first + 2, first + 3);
}
