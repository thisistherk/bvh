//
// Simple random number generator
//
// SPDX-License-Identifier: MIT
// This code is licensed under MIT license (see LICENSE file for details)
//

#pragma once

#include <cstdint>
#include <string>


// Random number generator (www.pcg-random.org)

class Random
{
public:
    Random(uint64_t seed = 0x0123456789abcdefULL, uint64_t sequence = 0xfedcba9876543210ULL)
        : _state(seed)
        , _sequence(sequence)
    {
        for (uint32_t ii = 0; ii < 4; ii++)
            next_uint();
    }


    uint32_t next_uint()
    {
        const uint64_t M = 6364136223846793005ULL;

        uint64_t s = _state;
        uint64_t q = _sequence;

        _state = s * M + q;

        uint32_t x = (uint32_t)(((s >> 18U) ^ s) >> 27U);
        uint32_t y = (uint32_t)(s >> 59U);

        return rotate(x, y);
    }

    float next_float()
    {
        uint32_t x = next_uint();
        uint32_t y = (x >> 9) | 0x3f800000U;

        float z;
        std::memcpy(&z, &y, sizeof(float));

        return (z - 1.0f);
    }

private:
    uint32_t rotate(uint32_t x, uint32_t y)
    {
        return (y == 0) ? x : (x >> y) | (x << (32 - y));
    }

private:
    uint64_t _state;
    uint64_t _sequence;
};
