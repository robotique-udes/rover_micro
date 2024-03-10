#ifndef __ASSERT_HPP__
#define __ASSERT_HPP__

#if defined(VERBOSE)
#include "helpers/log.hpp"

#define GET_MACRO_ASSERT(_0, _1, _2, ASSERT, ...) ASSERT
#define ASSERT(...) GET_MACRO_ASSERT(_0, ##__VA_ARGS__, ASSERT2, ASSERT1, ASSERT0)(__VA_ARGS__)

#define ASSERT2(condition, ...)      \
    {                                \
        if (condition)               \
        {                            \
            STOP_ALL_TIMERS()        \
            LOG(FATAL, __VA_ARGS__); \
            Serial.flush();          \
            abort();                 \
        }                            \
    }

#define ASSERT1(condition)                \
    {                                     \
        if (condition)                    \
        {                                 \
            STOP_ALL_TIMERS()             \
            LOG(FATAL, "%s", #condition); \
            Serial.flush();               \
            abort();                      \
        }                                 \
    }

#define ASSERT0()         \
    {                     \
        STOP_ALL_TIMERS() \
        abort();          \
    }

#if defined(ESP32)
#include "driver/ledc.h"
// Stop all possible timer combination to make sure all motors/servo stops even
// if code execution is stopped by user.
#define STOP_ALL_TIMERS()                                                    \
    {                                                                        \
        for (uint8_t j = LEDC_HIGH_SPEED_MODE; j < LEDC_SPEED_MODE_MAX; j++) \
        {                                                                    \
            for (uint8_t i = LEDC_CHANNEL_0; i < LEDC_CHANNEL_MAX; i++)      \
            {                                                                \
                ledc_stop((ledc_mode_t)j, (ledc_channel_t)i, 0u);            \
            }                                                                \
        }                                                                    \
    }
#else
#define STOP_ALL_TIMERS()
#endif

#else // defined(VERBOSE)
#define ASSERT(condition) ? abort() : (void)0;
#endif // defined(VERBOSE)

#endif // __ASSERT_HPP__
