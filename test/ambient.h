//
// Produce a simple 'ambient occulsion' image
//
// SPDX-License-Identifier: MIT
// This code is licensed under MIT license (see LICENSE file for details)
//

#pragma once

#include "bvh/base.h"

#include "image.h"
#include "mesh.h"
#include "timer.h"
#include "v3.h"

#include <memory>


struct Camera
{
    v3    from;
    v3    to;
    v3    up;
    float fov;
};


struct Stats
{
    Timer    primary_timer;
    uint32_t primary_count;
    Timer    shadow_timer;
    uint32_t shadow_count;
};


class AmbientOcclusion
{
public:
    // Begin rendering an image
    void begin(Mesh* mesh, bvh::Base* bvh, uint32_t w, uint32_t h, const Camera& camera);

    // Refine the current image
    void refine();

    // Query the current image
    const Image* image()
    {
        return _image.get();
    }

    // Query stats
    const Stats* stats()
    {
        return &_stats;
    }

private:
    // Image being rendered
    std::unique_ptr<Image> _image;

    // Camera details
    v3 _origin;
    v3 _view_x;
    v3 _view_y;
    v3 _view_z;

    // Mesh to render
    Mesh* _mesh;

    // BVH for mesh
    bvh::Base* _bvh;

    // Timing stats
    Stats _stats;

    // Current sample
    uint32_t _sample;

    // Buffers for BVH trace
    struct Pixel
    {
        uint16_t x;
        uint16_t y;
    };
    std::vector<Pixel>    _pixels;
    std::vector<bvh::Ray> _rays;
    std::vector<bvh::Ray> _shadows;
    std::vector<bvh::Hit> _hits;
};
