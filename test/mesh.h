//
// Triangle mesh data
//
// SPDX-License-Identifier: MIT
// This code is licensed under MIT license (see LICENSE file for details)
//

#pragma once

#include "v3.h"

#include <cstdint>
#include <vector>


class Mesh
{
public:
    // Read OBJ file
    Mesh(const char* path);

    // Return vertex/triangle count
    uint32_t vertices() const;
    uint32_t triangles() const;

    // Position data
    const v3* positions() const;

    // Triangle index data
    const uint32_t* indices() const;

    // Return centre of bounds
    v3 centre() const;

    // Return radius of bounding sphere
    float radius() const;

    // Add a ground plane on the given axis
    void add_plane(uint32_t axis, float size);


private:
    void add_triangle(uint32_t a, uint32_t b, uint32_t c);

private:
    // Original bounding box (not including any ground plane)
    v3 _min;
    v3 _max;

    // Vertex data
    std::vector<v3> _positions;

    // Triangle data
    std::vector<uint32_t> _indices;
};


// Vertex count

inline uint32_t Mesh::vertices() const
{
    return static_cast<uint32_t>(_positions.size() / 3);
}


// Triangle count

inline uint32_t Mesh::triangles() const
{
    return static_cast<uint32_t>(_indices.size() / 3);
}


// Position data

inline const v3* Mesh::positions() const
{
    return _positions.data();
}


// Triangle index data

inline const uint32_t* Mesh::indices() const
{
    return _indices.data();
}
