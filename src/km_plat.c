/*
* Copyright (C) 2026 Fredrik Skogman, skogman - at - gmail.com.
*
* The contents of this file are subject to the terms of the Common
* Development and Distribution License (the "License"). You may not use this
* file except in compliance with the License. You can obtain a copy of the
* License at http://opensource.org/licenses/CDDL-1.0. See the License for the
* specific language governing permissions and limitations under the License.
* When distributing the software, include this License Header Notice in each
* file and include the License file at http://opensource.org/licenses/CDDL-1.0.
*/

#include <stdlib.h>
#include "km_plat.h"
#if __linux
# include <errno.h>
# include <stdint.h>
# include <sys/random.h>
#endif

float rand_u01(void)
{
#if __APPLE__
        return (float)arc4random() / (float)UINT32_MAX;
#elif __linux__
        uint32_t x;
        unsigned char *p = (unsigned char*)&x;
        size_t left = sizeof(x);

        while (left > 0)
        {
                ssize_t n = getrandom(p, left, 0);
                if (n < 0)
                {
                        if (errno == EINTR)
                        {
                                continue;
                        }
                        __builtin_trap();
                }
                p += (size_t)n;
                left -= (size_t)n;
        }

        return (float)x / (float)UINT32_MAX;
#else
# error "Unknown target OS"
#endif
}
