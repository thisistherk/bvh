//
// Utility code shared by BVH implementations
//
// SPDX-License-Identifier: MIT
// This code is licensed under MIT license (see LICENSE file for details)
//

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>


namespace bvh
{
    struct v3
    {
        float x;
        float y;
        float z;

        v3() = default;
        v3(float xx, float yy, float zz)
            : x(xx)
            , y(yy)
            , z(zz)
        {
        }
        v3(const float* xyz)
            : x(xyz[0])
            , y(xyz[1])
            , z(xyz[2])
        {
        }

        float operator[](uint32_t idx) const
        {
            return *(&x + idx);
        }
    };

    inline float dot(v3 a, v3 b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    inline v3 cross(v3 a, v3 b)
    {
        return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
    }

    inline v3 operator+(v3 a, v3 b)
    {
        return v3(a.x + b.x, a.y + b.y, a.z + b.z);
    }

    inline v3 operator-(v3 a, v3 b)
    {
        return v3(a.x - b.x, a.y - b.y, a.z - b.z);
    }

    inline v3 operator*(v3 a, float b)
    {
        return v3(a.x * b, a.y * b, a.z * b);
    }

    inline v3 operator*(float a, v3 b)
    {
        return b * a;
    }

    inline v3 operator/(v3 a, float b)
    {
        return v3(a.x / b, a.y / b, a.z / b);
    }

    inline v3 norm(v3 a)
    {
        return a * (1.0f / sqrtf(dot(a, a)));
    }

    inline v3 min(v3 a, v3 b)
    {
        return v3(std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z));
    }

    inline v3 max(v3 a, v3 b)
    {
        return v3(std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z));
    }

    inline uint32_t max_dim(v3 a)
    {
        if (fabsf(a.x) > fabsf(a.y))
        {
            if (fabsf(a.x) > fabsf(a.z))
                return 0;
            else
                return 2;
            ;
        }
        else
        {
            if (fabsf(a.y) > fabsf(a.z))
                return 1;
            else
                return 2;
            ;
        }
    }


    struct WoopRay
    {
        v3       org;
        float    Sx;
        float    Sy;
        float    Sz;
        uint32_t x_index;
        uint32_t y_index;
        uint32_t z_index;
    };


    inline WoopRay woop_ray(v3 org, v3 dir)
    {
        WoopRay r;

        r.org = org;

        r.z_index = max_dim(dir);
        r.x_index = (r.z_index + 1) % 3;
        r.y_index = (r.z_index + 2) % 3;

        if (dir[r.z_index] < 0.0f)
            std::swap(r.x_index, r.y_index);

        r.Sx = dir[r.x_index] / dir[r.z_index];
        r.Sy = dir[r.y_index] / dir[r.z_index];
        r.Sz = 1.0f / dir[r.z_index];

        return r;
    }


    inline bool woop_ray_vs_triangle(WoopRay r, float min_t, float max_t, v3 p0, v3 p1, v3 p2, float* bary, float* d)
    {
        const v3 A = p0 - r.org;
        const v3 B = p1 - r.org;
        const v3 C = p2 - r.org;

        const float Ax = A[r.x_index] - r.Sx * A[r.z_index];
        const float Ay = A[r.y_index] - r.Sy * A[r.z_index];
        const float Bx = B[r.x_index] - r.Sx * B[r.z_index];
        const float By = B[r.y_index] - r.Sy * B[r.z_index];
        const float Cx = C[r.x_index] - r.Sx * C[r.z_index];
        const float Cy = C[r.y_index] - r.Sy * C[r.z_index];

        float U = Cx * By - Cy * Bx;
        float V = Ax * Cy - Ay * Cx;
        float W = Bx * Ay - By * Ax;

        if (U == 0.0f || V == 0.0f || W == 0.0f)
        {
            double CxBy = (double)Cx * (double)By;
            double CyBx = (double)Cy * (double)Bx;
            U           = (float)(CxBy - CyBx);
            double AxCy = (double)Ax * (double)Cy;
            double AyCx = (double)Ay * (double)Cx;
            V           = (float)(AxCy - AyCx);
            double BxAy = (double)Bx * (double)Ay;
            double ByAx = (double)By * (double)Ax;
            W           = (float)(BxAy - ByAx);
        }

        if ((U < 0.0f || V < 0.0f || W < 0.0f) && (U > 0.0f || V > 0.0f || W > 0.0f))
            return false;

        float det = U + V + W;
        if (det == 0.0f)
            return false;

        const float Az = r.Sz * A[r.z_index];
        const float Bz = r.Sz * B[r.z_index];
        const float Cz = r.Sz * C[r.z_index];
        const float T  = U * Az + V * Bz + W * Cz;

        const float rcp_det = 1.0f / det;
        float       t       = T * rcp_det;
        if (t < min_t || t > max_t)
            return false;

        *d      = t;
        bary[0] = V * rcp_det;
        bary[1] = W * rcp_det;

        return true;
    }


    inline bool ray_vs_bounds(v3 org, v3 inv_dir, float tmin, float tmax, v3 min, v3 max)
    {
        float tx1 = (min.x - org.x) * inv_dir.x;
        float tx2 = (max.x - org.x) * inv_dir.x;

        tmin = std::max(tmin, std::min(tx1, tx2));
        tmax = std::min(tmax, std::max(tx1, tx2));

        float ty1 = (min.y - org.y) * inv_dir.y;
        float ty2 = (max.y - org.y) * inv_dir.y;

        tmin = std::max(tmin, std::min(ty1, ty2));
        tmax = std::min(tmax, std::max(ty1, ty2));

        float tz1 = (min.z - org.z) * inv_dir.z;
        float tz2 = (max.z - org.z) * inv_dir.z;

        tmin = std::max(tmin, std::min(tz1, tz2));
        tmax = std::min(tmax, std::max(tz1, tz2));

        return (tmax >= tmin);
    }


    inline bool ray_vs_triangle(v3 org, v3 dir, float min_t, float max_t, v3 p0, v3 p1, v3 p2, float* bary, float* d)
    {
        const float EPSILON = 1.0e-5f;

        v3 edge1 = p1 - p0;
        v3 edge2 = p2 - p0;

        v3    h = cross(dir, edge2);
        float a = dot(edge1, h);

        if (a > -EPSILON && a < EPSILON)
            return false; // This ray is parallel to this triangle.

        float f = 1.0f / a;

        v3    s = org - p0;
        float u = f * dot(s, h);
        if (u < 0.0 || u > 1.0)
            return false;

        v3    q = cross(s, edge1);
        float v = f * dot(dir, q);
        if (v < 0.0 || u + v > 1.0)
            return false;

        // At this stage we can compute t to find out where the intersection point is on the line.
        float t = f * dot(edge2, q);
        if (t < min_t || t > max_t)
            return false;

        bary[0] = u;
        bary[1] = v;
        *d      = t;

        return true;
    }
}
