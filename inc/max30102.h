/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-02-08     Jackistang   the first version
 */
#ifndef PKG_MAX30102_H__
#define PKG_MAX30102_H__

#include <rtthread.h>
#include <rtdevice.h>

#ifndef MAX30102_STACK_SIZE
#define MAX30102_STACK_SIZE 1024
#endif

#ifndef MAX30102_PRIORITY
#define MAX30102_PRIORITY   10
#endif

#ifndef MAX30102_TICKS
#define MAX30102_TICKS      10
#endif

#if defined(RT_VERSION_CHECK)
    #if (RTTHREAD_VERSION >= RT_VERSION_CHECK(5, 0, 2))
        #define RT_SIZE_TYPE   rt_ssize_t
    #else
        #define RT_SIZE_TYPE   rt_size_t
    #endif
#endif

int rt_hw_max30102_init(struct rt_sensor_config *cfg);

#endif /*  MAX30102_H_ */
