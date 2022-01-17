//
// Greyscale image that can be written to file
//
// SPDX-License-Identifier: MIT
// This code is licensed under MIT license (see LICENSE file for details)
//

#pragma once

#include <cstdint>
#include <vector>


class Image
{
public:
    Image() = default;
    Image(uint32_t w, uint32_t h);

    void zero();
    bool write(const char* path) const;

    void add(uint32_t x, uint32_t y, float val)
    {
        _data[x + y * _width] += val;
    }

    uint32_t width() const
    {
        return _width;
    }

    uint32_t height() const
    {
        return _height;
    }


private:
    // Dimensions
    uint32_t _width;
    uint32_t _height;

    // Data
    std::vector<float> _data;
};
