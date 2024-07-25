#ifndef __ASSERT_HPP__
#define __ASSERT_HPP__

// =============================================================================
// assert.hpp defines macro which can be used to block code execution.
// 3 Versions exists:
//      ASSERT():
//          Block code execution but doesn't log anything, can be useful for
//          debugging, but other versions should be prefered in your codes
//
//      ASSERT(condition):
//          Check the condition. If it's true, the condition if logged and then
//          the code execution is stopped.
//
//      ASSERT(condition, ...):
//          Check the condition. If it's true, the custom msg "..." is logged
//          and then the code execution is stopped.
//
//      Usage Example:
//          void setup (void)
//          {
//              int* intPointer = NULL;
//              ASSERT(intPointer == NULL);
//          }
//
//          This will print the following: [FATAL][...]"intPointer == NULL"
//          and will cancel code execution
//
// On certain boards, assertion will reboot the controller instead of calling a
// for(;;) loop.
// =============================================================================

#if defined(VERBOSE)
#include "rover_helpers/log.hpp"
#include "driver/ledc.h"

// Stoping every pwm output to stop everything before aborting
#define ABORT()                                                                        \
    for (int speed_mode = 0; speed_mode < (int)LEDC_SPEED_MODE_MAX; speed_mode++)      \
    {                                                                                  \
        for (int channel = LEDC_CHANNEL_0; channel < (int)LEDC_CHANNEL_MAX; channel++) \
        {                                                                              \
            ledc_stop((ledc_mode_t)speed_mode, (ledc_channel_t)channel, 0);            \
        }                                                                              \
    }                                                                                  \
    Serial.flush();                                                                    \
    abort();

#define GET_MACRO_ASSERT(_0, _1, _2, ASSERT, ...) ASSERT
#define ASSERT(...) GET_MACRO_ASSERT(_0, ##__VA_ARGS__, ASSERT_CONDITION_MESSAGE, ASSERT_CONDITION, ASSERT_ONLY)(__VA_ARGS__)

#define ASSERT_CONDITION_MESSAGE(condition, ...) \
    {                                            \
        if (condition)                           \
        {                                        \
            LOG(FATAL, __VA_ARGS__);             \
            Serial.flush();                      \
            ABORT();                             \
        }                                        \
    }

#define ASSERT_CONDITION(condition)       \
    {                                     \
        if (condition)                    \
        {                                 \
            LOG(FATAL, "%s", #condition); \
            Serial.flush();               \
            ABORT();                      \
        }                                 \
    }

#define ASSERT_ONLY() \
    {                 \
        ABORT();      \
    }

#else // defined(VERBOSE)
#define ASSERT(condition, ...) condition ? abort() : (void)0;
#endif // defined(VERBOSE)

#endif // __ASSERT_HPP__
