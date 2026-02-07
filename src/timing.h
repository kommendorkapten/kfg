/*
* Copyright (C) 2016 Fredrik Skogman, skogman - at - gmail.com.
*
* The contents of this file are subject to the terms of the Common
* Development and Distribution License (the "License"). You may not use this
* file except in compliance with the License. You can obtain a copy of the
* License at http://opensource.org/licenses/CDDL-1.0. See the License for the
* specific language governing permissions and limitations under the License.
* When distributing the software, include this License Header Notice in each
* file and include the License file at http://opensource.org/licenses/CDDL-1.0.
*/

#ifndef __TIMING_H__
#define __TIMING_H__

#include <stdlib.h>
#include <sys/time.h>

struct timing
{
        struct timeval tv;
};

/**
 * Start a timer object.
 * @param the timing struct to initialize.
 * @return void.
 */
extern void timing_start(struct timing*);

/**
 * Extract the duration from the time to the current time. Returned
 * value is truncated.
 * @param the start time.
 * @return the duration since the start time in seconds.
 */
extern long timing_dur_sec(const struct timing*);

/**
 * Extract the duration from the time to the current time.
 * @param the start time.
 * @return the duration since the start time in micro seconds.
 */
extern long timing_dur_usec(const struct timing*);

/**
 * Extract the duration from the time to the current time. Returned
 * value is truncated.
 * @param the start time.
 * @return the duration since the start time in milli seconds.
 */
extern long timing_dur_msec(const struct timing*);

/**
 * Return the current epoch time in milli seconds.
 * @param void
 * @return epoch time in milli seconds.
 */
extern long timing_current_millis(void);

/**
 * Return the current epoch time in micro seconds.
 * @param void
 * @return epoch time in micro seconds.
 */
extern long timing_current_usec(void);

/**
 * Sleep using a hybrid mode. Uses nanosleep(2) for 80% of the time,
 * then busy waiting for the rest of the time.
 * This usually provides much better precision on some systems.
 * @params ns the time to sleep in nanoseconds.
 */
extern void timing_sleep(long ns);

#endif /* __TIMING_H__ */
