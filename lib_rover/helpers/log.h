#ifndef __LOG_H__
#define __LOG_H__

#include "helpers/logger.h"

#if defined(VERBOSE) && defined(WITH_MICRO_ROS)
// Args: (Logger::eLoggerLevel), (string literal)NodeName, (string literal)format, vars
#define LOG(severity, ...)                                                                      \
    {                                                                                           \
        if (Logger.isAlive())                                                                   \
        {                                                                                       \
            Logger.log(severity, __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__);            \
        }                                                                                       \
        else                                                                                    \
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
        }                                                                                       \
    }

#elif !defined(WITH_MICRO_ROS)
#define LOG(severity, ...)                                                                  \
    {                                                                                       \
        if (severity >= LOGGER_LOWEST_LEVEL)                                                 \
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

#else // defined(VERBOSE) && defined(WITH_MICRO_ROS)
#define LOG(severity, ...)

#endif // defined(VERBOSE) && defined(WITH_MICRO_ROS)

#endif //__LOG_H__
