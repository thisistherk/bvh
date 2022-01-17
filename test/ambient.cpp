//
// Simple path tracer
//
// SPDX-License-Identifier: MIT
// This code is licensed under MIT license (see LICENSE file for details)
//

#include "ambient.h"

#include "random.h"
#include "v3.h"


namespace
{
    void basis(v3 n, v3* x, v3* y)
    {
        v3 v;
        if (std::abs(n[0]) > std::abs(n[1]))
            v = cross(n, v3(0, 1, 0));
        else
            v = cross(n, v3(1, 0, 0));

        *x = norm(cross(v, n));
        *y = norm(cross(n, *x));
    }


    // Bit cast between types

    template <typename T, typename F>
    T bit_cast(F x)
    {
        static_assert(sizeof(T) == sizeof(F));

        T y;
        memcpy(&y, &x, sizeof(x));
        return y;
    }


    // Offset a ray origin to avoid self intersections (RTGems I, Chapter 6)

    v3 offset_origin(v3 p, v3 n)
    {
        constexpr float origin      = 1.0 / 32.0;
        constexpr float float_scale = 1.0 / 65536.0;
        constexpr float int_scale   = 256.0;

        int i0 = static_cast<int>(int_scale * n[0]);
        int i1 = static_cast<int>(int_scale * n[1]);
        int i2 = static_cast<int>(int_scale * n[2]);

        float p0 = bit_cast<float>(bit_cast<int>(p[0]) + ((p[0] < 0.0) ? -i0 : i0));
        float p1 = bit_cast<float>(bit_cast<int>(p[1]) + ((p[1] < 0.0) ? -i1 : i1));
        float p2 = bit_cast<float>(bit_cast<int>(p[2]) + ((p[2] < 0.0) ? -i2 : i2));

        return v3((std::abs(p[0]) < origin) ? (p[0] + float_scale * n[0]) : p0,
                  (std::abs(p[1]) < origin) ? (p[1] + float_scale * n[1]) : p1,
                  (std::abs(p[2]) < origin) ? (p[2] + float_scale * n[2]) : p2);
    }
}


// Begin rendering an image

void AmbientOcclusion::begin(Mesh* mesh, bvh::Base* bvh, uint32_t w, uint32_t h, const Camera& camera)
{
    // Store info
    _image = std::make_unique<Image>(w, h);
    _mesh  = mesh;
    _bvh   = bvh;

    // Reset sample number
    _sample = 0;

    // Reset stats
    _stats.primary_timer = Timer();
    _stats.primary_count = 0;

    // Camera info
    float scale  = std::tanf(0.5f * camera.fov);
    float aspect = static_cast<float>(h) / static_cast<float>(w);

    _origin = camera.from;
    _view_z = norm(camera.to - camera.from);
    _view_x = scale * norm(cross(_view_z, camera.up));
    _view_y = aspect * scale * norm(cross(_view_z, _view_x));

    // Create buffers used during trace
    uint32_t pixels = w * h;
    _pixels.resize(pixels);
    _rays.resize(pixels);
    _shadows.resize(pixels);
    _hits.resize(pixels);
}


// Refine the current image

void AmbientOcclusion::refine()
{
    uint32_t s = _sample++;
    Random   rnd(s);


    // Generate camera rays
    uint32_t w = _image->width();
    uint32_t h = _image->height();

    uint32_t ray_count = 0;
    for (uint32_t y = 0; y < h; y++)
    {
        for (uint32_t x = 0; x < w; x++)
        {
            Pixel pixel;
            pixel.x            = static_cast<uint16_t>(x);
            pixel.y            = static_cast<uint16_t>(y);
            _pixels[ray_count] = pixel;

            float fx = rnd.next_float();
            float fy = rnd.next_float();

            float sx = 2.0f * (static_cast<float>(x) + fx) / static_cast<float>(w) - 1.0f;
            float sy = 2.0f * (static_cast<float>(y) + fy) / static_cast<float>(h) - 1.0f;

            v3 d = norm(_view_z + sx * _view_x + sy * _view_y);

            bvh::Ray ray;
            ray.origin[0] = _origin[0];
            ray.origin[1] = _origin[1];
            ray.origin[2] = _origin[2];

            ray.direction[0] = d[0];
            ray.direction[1] = d[1];
            ray.direction[2] = d[2];

            ray.min_t = 0.0f;
            ray.max_t = INFINITY;

            _rays[ray_count] = ray;

            ray_count++;
        }
    }


    // Trace them
    _stats.primary_count += w * h;
    _stats.primary_timer.begin();

    _bvh->trace(ray_count, _rays.data(), _hits.data(), bvh::TRACE_COHERENT);

    _stats.primary_timer.end();


    // Generate shadow rays
    const v3*       ps = _mesh->positions();
    const uint32_t* is = _mesh->indices();

    uint32_t shadow_count = 0;
    for (uint32_t ray_idx = 0; ray_idx < ray_count; ray_idx++)
    {
        auto hit = _hits[ray_idx];
        if (hit.triangle != bvh::TRIANGLE_INVALID)
        {
            auto pixel = _pixels[ray_idx];

            v3 d(_rays[ray_idx].direction);

            uint32_t i0 = is[3 * hit.triangle + 0];
            uint32_t i1 = is[3 * hit.triangle + 1];
            uint32_t i2 = is[3 * hit.triangle + 2];

            v3 p0 = ps[i0];
            v3 p1 = ps[i1];
            v3 p2 = ps[i2];

            float v = hit.barycentric[0];
            float w = hit.barycentric[1];
            float u = 1.0f - (v + w);

            v3 p = u * p0 + v * p1 + w * p2;

            v3 p01 = p1 - p0;
            v3 p02 = p2 - p0;
            v3 n   = norm(cross(p01, p02));

            if (dot(n, d) > 0.0)
                n = -n;

            v3 x;
            v3 y;
            basis(n, &x, &y);

            float cos_theta = 1.0f - rnd.next_float();
            float sin_theta = std::sqrtf(1.0f - cos_theta * cos_theta);
            float phi       = 2.0f * M_PI * rnd.next_float();
            float cos_phi   = std::cos(phi);
            float sin_phi   = std::sin(phi);

            v3 r = cos_phi * sin_theta * x + sin_phi * sin_theta * y + cos_theta * n;
            v3 o = offset_origin(p, n);

            bvh::Ray shadow;
            shadow.origin[0]    = o[0];
            shadow.origin[1]    = o[1];
            shadow.origin[2]    = o[2];
            shadow.direction[0] = r[0];
            shadow.direction[1] = r[1];
            shadow.direction[2] = r[2];
            shadow.min_t        = 1.0e-4f;
            shadow.max_t        = INFINITY;

            _shadows[shadow_count] = shadow;
            _pixels[shadow_count]  = pixel;

            shadow_count++;
        }
    }


    // Trace shadow rays
    _stats.shadow_count += shadow_count;
    _stats.shadow_timer.begin();

    _bvh->trace(shadow_count, _shadows.data(), _hits.data(), bvh::TRACE_SHADOW);

    _stats.shadow_timer.end();


    // Acculumate in image
    for (uint32_t shadow_idx = 0; shadow_idx < shadow_count; shadow_idx++)
    {
        if (_hits[shadow_idx].triangle == bvh::TRIANGLE_INVALID)
        {
            Pixel pixel = _pixels[shadow_idx];
            _image->add(pixel.x, pixel.y, 1.0);
        }
    }
}
