#ifndef __LOGGER_HPP__
#define __LOGGER_HPP__

#if !defined (ESP32)
#error CPU not supported
#endif

#if defined(MICRO_ROS_LOGGER)

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl_interfaces/msg/log.h>
#include "helpers/helpers.hpp"

// Make sure code will execute fine even if LOGGER_LOWEST_LEVEL isn't defined
#ifndef LOGGER_LOWEST_LEVEL
#define LOGGER_LOWEST_LEVEL 0
#endif // LOGGER_LOWEST_LEVEL

    // This class creates a publisher and publish msgs to rosout to log msgs the ROS
    // terminal running the node
    typedef enum eLoggerLevel
    {
        DEBUG = rcl_interfaces__msg__Log__DEBUG,
        INFO = rcl_interfaces__msg__Log__INFO,
        WARN = rcl_interfaces__msg__Log__WARN,
        ERROR = rcl_interfaces__msg__Log__ERROR,
        FATAL = rcl_interfaces__msg__Log__FATAL
    } eLoggerLevel;

namespace RoverMicroRosLib
{
    class Logger
    {
    public:
        Logger(void);
        ~Logger(void);

        bool createLogger(rcl_node_t *node_, const char *nodeName_, const char *ns_);
        void destroyLogger(rcl_node_t *node_);
        void log(eLoggerLevel lvl_, const char *file_, const char *function_, int line_, const char *str_, ...);
        bool isAlive(void);

    private:
        rcl_publisher_t _pubLogger;
        bool _alive = false;

        const char *_nodeName = NULL;
        const char *_ns = NULL;
    };

    extern Logger G_Logger;
}

#endif // MICRO_ROS_LOGGER
#endif // __LOGGER_HPP__
