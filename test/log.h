//
// Log output
//
// SPDX-License-Identifier: MIT
// This code is licensed under MIT license (see LICENSE file for details)
//

#pragma once

#include <cstdarg>
#include <cstdio>
#include <cstdlib>


// Output a log message

inline void log(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);

    printf("\n");
}


// Output an error message and exit

inline void error(const char* fmt, ...)
{
    char err[1024];

    va_list args;
    va_start(args, fmt);
    vsnprintf(err, sizeof(err), fmt, args);
    va_end(args);

    err[sizeof(err) - 1] = '\0';

    log("ERROR: %s", err);
    std::exit(-1);
}
