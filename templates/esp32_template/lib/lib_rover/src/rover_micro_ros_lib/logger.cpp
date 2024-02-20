#include "rover_micro_ros_lib/logger.hpp"

namespace RoverMicroRosLib
{
    Logger G_Logger;

    Logger::Logger(void)
    {
    }

    Logger::~Logger(void)
    {
    }

    bool Logger::createLogger(rcl_node_t *node_, const char *nodeName_, const char *ns_)
    {
        _nodeName = nodeName_;
        _ns = ns_;

        RCLC_RET_ON_ERR(rclc_publisher_init_default(&_pubLogger,
                                                    node_,
                                                    ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
                                                    NAME_LOG_TOPIC));

        _alive = true;
        return true;
    }

    void Logger::destroyLogger(rcl_node_t *node_)
    {
        _alive = false;
        REMOVE_WARN_UNUSED(rcl_publisher_fini(&_pubLogger, node_));
    }

    void Logger::log(eLoggerLevel lvl_, const char *file_, const char *function_, int line_, const char *str_, ...)
    {
        if (lvl_ < LOGGER_LOWEST_LEVEL)
        {
            return;
        }
        rcl_interfaces__msg__Log msg;

        msg.level = lvl_;

        uint8_t buffer_size = 0;
        if (_nodeName == NULL || _ns == NULL)
        {
            buffer_size = sizeof("node_name_not_set");
        }
        else
        {
            buffer_size = strlen(_ns) + 1 + strlen(_nodeName);
        }

        char buffer[buffer_size] = "\0";
        if (_nodeName == NULL || _ns == NULL)
        {
            strncat(buffer, "node_name_not_set", sizeof("node_name_not_set"));
        }
        else
        {
            strncat(buffer, _ns, strlen(_ns) + 1);
            strncat(buffer, "/", sizeof("/"));
            strncat(buffer, _nodeName, strlen(_nodeName) + 1);
        }
        msg.name.data = buffer;

        msg.name.size = strlen(msg.name.data);

        msg.file.data = (char *)file_;
        msg.file.size = strlen(msg.file.data);

        msg.function.data = (char *)function_;
        msg.function.size = strlen(msg.function.data);

        msg.line = line_;

        // Buffer size is arbitrairy now,
        // TODO  might want to evaluate actual necessary

        va_list strArgs;
        va_start(strArgs, str_);
        char msgBuffer[snprintf(NULL, 0, str_, strArgs) + 1];

        vsnprintf(msgBuffer, sizeof(msgBuffer), str_, strArgs);
        msg.msg.data = (char *)msgBuffer;
        msg.msg.size = strlen(msg.msg.data);

        va_end(strArgs);

        REMOVE_WARN_UNUSED(rcl_publish(&_pubLogger, &msg, NULL));
    }

    bool Logger::isAlive(void)
    {
        return _alive;
    }
}
