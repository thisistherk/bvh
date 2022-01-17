//
// Simple 3D vector
//
// SPDX-License-Identifier: MIT
// This code is licensed under MIT license (see LICENSE file for details)
//

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>


class v3
{
public:
    v3() = default;
    v3(float x, float y, float z)
        : _data{x, y, z}
    {
    }

    v3(const float* ptr)
        : _data{ptr[0], ptr[1], ptr[2]}
    {
    }

    float& operator[](uint32_t idx)
    {
        return _data[idx];
    }

    float operator[](uint32_t idx) const
    {
        return _data[idx];
    }

private:
    float _data[3];
};


inline v3 operator-(v3 a)
{
    return v3(-a[0], -a[1], -a[2]);
}


inline v3 operator+(v3 a, v3 b)
{
    return v3(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}


inline v3 operator-(v3 a, v3 b)
{
    return v3(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}


inline v3 operator*(float a, v3 b)
{
    return v3(a * b[0], a * b[1], a * b[2]);
}


inline v3 operator*(v3 a, float b)
{
    return v3(a[0] * b, a[1] * b, a[2] * b);
}


inline v3 operator/(v3 a, float b)
{
    return a * (1.0f / b);
}


inline float dot(v3 a, v3 b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}


inline v3 cross(v3 a, v3 b)
{
    return v3(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
}


inline float len(v3 a)
{
    return std::sqrtf(dot(a, a));
}


inline v3 norm(v3 a)
{
    return a / len(a);
}


inline v3 min(v3 a, v3 b)
{
    return v3(std::min(a[0], b[0]), std::min(a[1], b[1]), std::min(a[2], b[2]));
}


inline v3 max(v3 a, v3 b)
{
    return v3(std::max(a[0], b[0]), std::max(a[1], b[1]), std::max(a[2], b[2]));
}
