#ifndef __LOG_H__
#define __LOG_H__

// =============================================================================
// log.hpp helps redirecting log to either microRos or the Serial or to not
// compiled them at all for performance enhancement
//
//  Usage example:
//      void setup(void)
//      {
//          LOG(INFO, "This is a log message");
//      }
// =============================================================================

#if defined(VERBOSE)

#if defined(MICRO_ROS_LOGGER)
#if !defined(ESP32)
#error CPU not supported
#endif

#include "rover_micro_ros_lib/logger.hpp"
// Args: (Logger::eLoggerLevel), (string literal)NodeName, (string literal)format, vars
#define LOG(severity, ...)                                                                               \
    {                                                                                                    \
        if (RoverMicroRosLib::G_Logger.isAlive())                                                        \
        {                                                                                                \
            RoverMicroRosLib::G_Logger.log(severity, __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__); \
        }                                                                                                \
        else                                                                                             \
        {                                                                                                \
            if (severity > LOGGER_LOWEST_LEVEL)                                                          \
            {                                                                                            \
                char severityStr[6] = "0_0";                                                             \
                char colorStr[8] = "\033[97m";                                                           \
                switch (severity)                                                                        \
                {                                                                                        \
                case DEBUG:                                                                              \
                    strcpy(severityStr, "DEBUG");                                                        \
                    break;                                                                               \
                                                                                                         \
                case INFO:                                                                               \
                    strcpy(severityStr, "INFO");                                                         \
                    break;                                                                               \
                                                                                                         \
                case WARN:                                                                               \
                    strcpy(severityStr, "WARN");                                                         \
                    strcpy(colorStr, "\033[33m");                                                        \
                    break;                                                                               \
                                                                                                         \
                case ERROR:                                                                              \
                    strcpy(severityStr, "ERROR");                                                        \
                    strcpy(colorStr, "\033[31m");                                                        \
                    break;                                                                               \
                                                                                                         \
                case FATAL:                                                                              \
                    strcpy(severityStr, "FATAL");                                                        \
                    strcpy(colorStr, "\033[31m");                                                        \
                    break;                                                                               \
                }                                                                                        \
                                                                                                         \
                Serial.printf("%s[%s]%s(%d): ", colorStr, severityStr, __FILENAME__, __LINE__);          \
                Serial.printf(__VA_ARGS__);                                                              \
                Serial.printf("\n\033[97m");                                                             \
            }                                                                                            \
        }                                                                                                \
    }

#else // defined(MICRO_ROS_LOGGER)

typedef enum eLoggerLevel
{
    DEBUG = 10,
    INFO = 20,
    WARN = 30,
    ERROR = 40,
    FATAL = 50
} eLoggerLevel;

#if !defined(ESP32)
#error CPU not supported
#endif
// Args: (Logger::eLoggerLevel), (string literal)NodeName, (string literal)format, vars
#define LOG(severity, ...)                                                                  \
    {                                                                                       \
        if (severity > LOGGER_LOWEST_LEVEL)                                                 \
        {                                                                                   \
            char severityStr[6] = "0_0";                                                    \
            char colorStr[8] = "\033[97m";                                                  \
            switch (severity)                                                               \
            {                                                                               \
            case DEBUG:                                                                     \
                strcpy(severityStr, "DEBUG");                                               \
                break;                                                                      \
                                                                                            \
            case INFO:                                                                      \
                strcpy(severityStr, "INFO");                                                \
                break;                                                                      \
                                                                                            \
            case WARN:                                                                      \
                strcpy(severityStr, "WARN");                                                \
                strcpy(colorStr, "\033[33m");                                               \
                break;                                                                      \
                                                                                            \
            case ERROR:                                                                     \
                strcpy(severityStr, "ERROR");                                               \
                strcpy(colorStr, "\033[31m");                                               \
                break;                                                                      \
                                                                                            \
            case FATAL:                                                                     \
                strcpy(severityStr, "FATAL");                                               \
                strcpy(colorStr, "\033[31m");                                               \
                break;                                                                      \
            }                                                                               \
                                                                                            \
            Serial.printf("%s[%s]%s(%d): ", colorStr, severityStr, __FILENAME__, __LINE__); \
            Serial.printf(__VA_ARGS__);                                                     \
            Serial.printf("\n\033[97m");                                                    \
        }                                                                                   \
    }
#endif // defined(MICRO_ROS_LOGGER)

template <typename T>
constexpr const char *variable_to_bit_string(T value)
{
    constexpr size_t bit_count = sizeof(T) * 8;
    static char bit_string[bit_count * 7 + 10] = {};

    char *current = bit_string;
    for (size_t i = 0; i < bit_count; ++i)
    {
        unsigned bit = (value >> (bit_count - 1 - i)) & 1;
        if (i == 0)
        {
            current += sprintf(current, "MSB[0x%u", bit);
        }
        else if (i == bit_count - 1)
        {
            current += sprintf(current, " | 0x%u|LSB]", bit);
        }
        else
        {
            current += sprintf(current, " | 0x%u", bit);
        }
    }
    return bit_string;
}

// Macro to log the bits of a variable using the Logger class
#define LOG_BITS(severity, var) \
    LOG(severity, "%s = %s", #var, variable_to_bit_string(var))

#else // VERBOSE
#define LOG(severity, ...)
#endif // VERBOSE

#endif //__LOG_H__
