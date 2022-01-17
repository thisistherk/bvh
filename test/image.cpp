//
// Greyscale image that can be written to file
//
// SPDX-License-Identifier: MIT
// This code is licensed under MIT license (see LICENSE file for details)
//

#include "image.h"

#include <cmath>
#include <cstdint>
#include <cstdio>


namespace
{
    void write_u16(FILE* f, uint16_t v)
    {
        fwrite(&v, sizeof(uint16_t), 1, f);
    }

    void write_u32(FILE* f, uint32_t v)
    {
        fwrite(&v, sizeof(uint32_t), 1, f);
    }
}


// Ctor

Image::Image(uint32_t w, uint32_t h)
    : _width(w)
    , _height(h)
    , _data(w * h)
{
}


// Zero the image

void Image::zero()
{
    std::memset(_data.data(), 0, _data.size() * sizeof(float));
}


// Write the image to bitmap

bool Image::write(const char* path) const
{
    FILE* f = fopen(path, "wb");
    if (!f)
        return false;

    constexpr uint32_t bfsize = 14;
    constexpr uint32_t bisize = 40;

    uint32_t bytes = 3 * _width * _height + bfsize + bisize;

    // File header
    write_u16(f, 0x4d42);          // bfType
    write_u32(f, bytes);           // bfSize
    write_u16(f, 0);               // bfReserved1
    write_u16(f, 0);               // bfReserved2
    write_u32(f, bfsize + bisize); // bfOffBits

    // Info header
    write_u32(f, bisize);  // biSize
    write_u32(f, _width);  // biWidth
    write_u32(f, _height); // biHeight
    write_u16(f, 1);       // biPlanes
    write_u16(f, 24);      // biBitCount
    write_u32(f, 0);       // biCompression
    write_u32(f, 0);       // biSizeImage
    write_u32(f, 0);       // biXPelsPerMeter
    write_u32(f, 0);       // biYPelsPerMeter
    write_u32(f, 0);       // biClrUsed
    write_u32(f, 0);       // biClrImportant

    // Max value in image
    float max = 0.0f;
    for (auto p : _data)
        max = std::max(max, p);

    float scale = 1.0f / max;


    // Write scans
    std::vector<uint8_t> scan(3 * _width);
    for (uint32_t y = 0; y < _height; y++)
    {
        const float* src = _data.data() + (_height - (y + 1)) * _width;
        uint8_t*     dst = scan.data();

        for (uint32_t x = 0; x < _width; x++)
        {
            float val = std::powf(scale * *src++, 1.0f / 2.4f);

            uint8_t v = static_cast<uint8_t>(val * 255.0f);

            *dst++ = v;
            *dst++ = v;
            *dst++ = v;
        }

        fwrite(scan.data(), 1, 3 * _width, f);
    }

    fclose(f);

    return true;
}
