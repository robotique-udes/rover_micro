#ifndef __LOG_H__
#define __LOG_H__

#if defined(VERBOSE)

#if defined(MICRO_ROS_LOGGER)
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

#else // VERBOSE
#define LOG(severity, ...)
#endif // VERBOSE

#endif //__LOG_H__
