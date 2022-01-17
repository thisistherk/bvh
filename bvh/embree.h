//
// Embree implementation for comparisons
//
// SPDX-License-Identifier: MIT
// This code is licensed under MIT license (see LICENSE file for details)
//

#pragma once

#if defined(BVH_EMBREE)

#include "bvh/base.h"

#include <embree3/rtcore.h>

#include <string>
#include <vector>


namespace bvh
{
    class Embree : public Base
    {
    public:
        Embree(RTCBuildQuality quality = RTC_BUILD_QUALITY_MEDIUM, int threads = 0)
            : _quality(quality)
        {
            std::string opts = "threads=" + std::to_string(threads);

            _device = rtcNewDevice(opts.c_str());
        }


        ~Embree()
        {
            rtcReleaseScene(_scene);
            rtcReleaseGeometry(_geometry);
            rtcReleaseDevice(_device);
        }


        void build(const Mesh* mesh) final
        {
            _scene    = rtcNewScene(_device);
            _geometry = rtcNewGeometry(_device, RTC_GEOMETRY_TYPE_TRIANGLE);

            rtcSetSceneBuildQuality(_scene, _quality);
            rtcSetSceneFlags(_scene, RTC_SCENE_FLAG_ROBUST);

            _vbo = static_cast<float*>(rtcSetNewGeometryBuffer(_geometry,
                                                               RTC_BUFFER_TYPE_VERTEX,
                                                               0,
                                                               RTC_FORMAT_FLOAT3,
                                                               3 * sizeof(float),
                                                               mesh->vertices));

            _ibo = static_cast<uint32_t*>(rtcSetNewGeometryBuffer(_geometry,
                                                                  RTC_BUFFER_TYPE_INDEX,
                                                                  0,
                                                                  RTC_FORMAT_UINT3,
                                                                  3 * sizeof(uint32_t),
                                                                  mesh->triangles));

            std::memcpy(_vbo, mesh->positions, 3 * mesh->vertices * sizeof(float));
            std::memcpy(_ibo, mesh->indices, 3 * mesh->triangles * sizeof(uint32_t));

            rtcCommitGeometry(_geometry);

            rtcAttachGeometry(_scene, _geometry);
            rtcCommitScene(_scene);
        }


        void trace(uint32_t rays, Ray* input, Hit* output, uint32_t flags) final
        {
            // Copy input data
            if (_origin_x.size() < rays)
            {
                _origin_x.resize(rays);
                _origin_y.resize(rays);
                _origin_z.resize(rays);
                _direction_x.resize(rays);
                _direction_y.resize(rays);
                _direction_z.resize(rays);
                _near.resize(rays);
                _far.resize(rays);
            }

            for (uint32_t ii = 0; ii < rays; ii++)
            {
                _origin_x[ii]    = input[ii].origin[0];
                _origin_y[ii]    = input[ii].origin[1];
                _origin_z[ii]    = input[ii].origin[2];
                _direction_x[ii] = input[ii].direction[0];
                _direction_y[ii] = input[ii].direction[1];
                _direction_z[ii] = input[ii].direction[2];
                _near[ii]        = input[ii].min_t;
                _far[ii]         = input[ii].max_t;
            }


            // Trace rays
            RTCIntersectContext ctx;
            rtcInitIntersectContext(&ctx);
            if (flags & TRACE_COHERENT)
                ctx.flags = RTC_INTERSECT_CONTEXT_FLAG_COHERENT;
            else
                ctx.flags = RTC_INTERSECT_CONTEXT_FLAG_INCOHERENT;

            if (flags & TRACE_SHADOW)
            {
                RTCRayNp data = {};
                data.org_x    = _origin_x.data();
                data.org_y    = _origin_y.data();
                data.org_z    = _origin_z.data();
                data.dir_x    = _direction_x.data();
                data.dir_y    = _direction_y.data();
                data.dir_z    = _direction_z.data();
                data.tnear    = _near.data();
                data.tfar     = _far.data();

                rtcOccludedNp(_scene, &ctx, &data, rays);

                // Copy results to output
                for (uint32_t ii = 0; ii < rays; ii++)
                {
                    if (data.tfar[ii] < 0)
                        output[ii].triangle = 0;
                    else
                        output[ii].triangle = TRIANGLE_INVALID;
                }
            }
            else
            {
                // Prepare output arrays
                if (_geom.size() < rays)
                {
                    _geom.resize(rays);
                    _prim.resize(rays);
                    _barycentric_u.resize(rays);
                    _barycentric_v.resize(rays);
                }

                for (uint32_t ii = 0; ii < rays; ii++)
                    _geom[ii] = RTC_INVALID_GEOMETRY_ID;


                RTCRayHitNp data = {};
                data.ray.org_x   = _origin_x.data();
                data.ray.org_y   = _origin_y.data();
                data.ray.org_z   = _origin_z.data();
                data.ray.dir_x   = _direction_x.data();
                data.ray.dir_y   = _direction_y.data();
                data.ray.dir_z   = _direction_z.data();
                data.ray.tnear   = _near.data();
                data.ray.tfar    = _far.data();

                data.hit.geomID = _geom.data();
                data.hit.primID = _prim.data();
                data.hit.u      = _barycentric_u.data();
                data.hit.v      = _barycentric_v.data();

                rtcIntersectNp(_scene, &ctx, &data, rays);

                // Copy results to output
                for (uint32_t ii = 0; ii < rays; ii++)
                {
                    if (_geom[ii] == RTC_INVALID_GEOMETRY_ID)
                    {
                        output[ii].triangle = TRIANGLE_INVALID;
                    }
                    else
                    {
                        output[ii].triangle       = _prim[ii];
                        output[ii].barycentric[0] = _barycentric_u[ii];
                        output[ii].barycentric[1] = _barycentric_v[ii];
                    }
                }
            }
        }


    private:
        // Embree scene data
        RTCDevice   _device;
        RTCScene    _scene;
        RTCGeometry _geometry;

        // BVH type
        RTCBuildQuality _quality;

        // Geometry data
        float*    _vbo;
        uint32_t* _ibo;

        // Intermediate buffers for ray input
        std::vector<float> _origin_x;
        std::vector<float> _origin_y;
        std::vector<float> _origin_z;
        std::vector<float> _direction_x;
        std::vector<float> _direction_y;
        std::vector<float> _direction_z;
        std::vector<float> _near;
        std::vector<float> _far;

        // Intermediate buffers for ray output
        std::vector<uint32_t> _geom;
        std::vector<uint32_t> _prim;
        std::vector<float>    _barycentric_u;
        std::vector<float>    _barycentric_v;
    };
}

#endif
