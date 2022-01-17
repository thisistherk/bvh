//
// Simple C++ timer
//
// SPDX-License-Identifier: MIT
// This code is licensed under MIT license (see LICENSE file for details)
//

#pragma once

#include <chrono>


class Timer
{
public:
    void begin()
    {
        _start = clock::now();
    }

    void end()
    {
        _total += clock::now() - _start;
    }

    double seconds() const
    {
        return std::chrono::duration<double>(_total).count();
    }

private:
    using clock = std::chrono::system_clock;

    clock::time_point _start;
    clock::duration   _total = std::chrono::duration<int>(0);
};
