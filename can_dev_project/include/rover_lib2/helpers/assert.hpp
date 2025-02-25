#ifndef ASSERT_HPP
#define ASSERT_HPP

#if !defined(ARDUINO_ESP32S3_DEV)
#error CPU not supported
#endif

#include "driver/ledc.h"
#include "rover_lib2/helpers/log.hpp"
#include <cstdarg>

DEFINE_LOG_NODE(ASSERTS, Logger::eNodeState::ON);

namespace
{
    constexpr void SHUTDOWN_PWM(void)
    {
        for (int speed_mode = 0; speed_mode < (int)LEDC_SPEED_MODE_MAX; speed_mode++)
        {
            for (int channel = 0; channel < (int)LEDC_CHANNEL_MAX; channel++)
            {
                ledc_stop((ledc_mode_t)speed_mode, (ledc_channel_t)channel, 0);
            }
        }
    }

    inline void ABORT(void)
    {
        SHUTDOWN_PWM();
        LOG_FLUSH();
        abort();
    }
}  // namespace

inline void ASSERT(void)
{
    LOG_FATAL(Logger::Nodes::ASSERTS, "Explicit assertion");
    ABORT();
}

inline void ASSERT(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    LOG_FATAL(Logger::Nodes::ASSERTS, "Explicit assertion: %s", format, args);
    va_end(args);
    ABORT();
}

inline void ASSERT(bool condition, const char* format, ...)
{
    if (!condition)
    {
        va_list args;
        va_start(args, format);
        LOG_FATAL(Logger::Nodes::ASSERTS, "Assertion failed: %s", format, args);
        va_end(args);
        ABORT();
    }
}

#endif  // ASSERT_HPP
