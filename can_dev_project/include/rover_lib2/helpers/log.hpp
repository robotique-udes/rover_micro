#ifndef __LOG_H__
#define __LOG_H__

#if defined(VERBOSE)

#if !defined(ARDUINO_ESP32S3_DEV)
#error CPU not supported
#endif

#include "rover_lib2/helpers/macros.hpp"
#include <Stream.h>

#if !defined(GLOBAL_SEVERITY_LEVEL)
#define GLOBAL_SEVERITY_LEVEL Logger::eSeverityLevels::DEBUG_
#endif

#if !defined(NODE_BYPASS_SEVERITY_LEVEL)
#define NODE_BYPASS_SEVERITY_LEVEL Logger::eSeverityLevels::INFO
#endif

#define DEFINE_LOG_NODE(name_, state_)                              \
    namespace Logger::Nodes                                         \
    {                                                               \
        inline constexpr Logger::Nodes::Node name_{#name_, state_}; \
    }

namespace Logger
{
    Stream& loggerStream __attribute__((weak)) = Serial;

    enum class eSeverityLevels : uint8_t
    {
        DEBUG_ = 0,
        INFO,
        WARN,
        ERROR,
        FATAL
    };

    enum class eNodeState
    {
        ON,
        OFF
    };

    namespace Nodes
    {
        struct Node
        {
            constexpr Node(const char* name_, eNodeState state_): name(name_), state(state_) {}
            const char* const name;
            const eNodeState state;
        };
    }  // namespace Nodes

    namespace
    {
        constexpr const char* SEVERITY_NAMES[] = {"DEBUG", "INFO", "WARN", "ERROR", "FATAL"};
        constexpr const char* SEVERITY_COLORS[] = {"\033[90m", "\033[97m", "\033[33m", "\033[31m", "\033[31m"};
        constexpr const char* COLOR_RESET = "\033[97m";

        template<eSeverityLevels SEVERITY>
        void logInternal(const Nodes::Node& node, const char* format, va_list args)
        {
            if (TO_UNDERLYING(SEVERITY) >= static_cast<size_t>(GLOBAL_SEVERITY_LEVEL)
                && (TO_UNDERLYING(SEVERITY) >= static_cast<size_t>(NODE_BYPASS_SEVERITY_LEVEL)
                    || node.state == Logger::eNodeState::ON))
            {
                const auto severityIndex = TO_UNDERLYING(SEVERITY);
                loggerStream.printf("%s[%lu][%s][%s]: ",
                                    SEVERITY_COLORS[severityIndex],
                                    millis(),
                                    SEVERITY_NAMES[severityIndex],
                                    node.name);
                loggerStream.vprintf(format, args);
                loggerStream.printf("\n%s", COLOR_RESET);
            }
        }
    }  // namespace
}  // namespace Logger

namespace LOG
{
    inline void DEBUG(const Logger::Nodes::Node& node, const char* format, ...)
    {
        va_list args;
        va_start(args, format);
        Logger::logInternal<Logger::eSeverityLevels::DEBUG_>(node, format, args);
        va_end(args);
    }

    inline void INFO(const Logger::Nodes::Node& node, const char* format, ...)
    {
        va_list args;
        va_start(args, format);
        Logger::logInternal<Logger::eSeverityLevels::INFO>(node, format, args);
        va_end(args);
    }

    inline void WARN(const Logger::Nodes::Node& node, const char* format, ...)
    {
        va_list args;
        va_start(args, format);
        Logger::logInternal<Logger::eSeverityLevels::WARN>(node, format, args);
        va_end(args);
    }

    inline void ERROR(const Logger::Nodes::Node& node, const char* format, ...)
    {
        va_list args;
        va_start(args, format);
        Logger::logInternal<Logger::eSeverityLevels::ERROR>(node, format, args);
        va_end(args);
    }

    inline void FATAL(const Logger::Nodes::Node& node, const char* format, ...)
    {
        va_list args;
        va_start(args, format);
        Logger::logInternal<Logger::eSeverityLevels::FATAL>(node, format, args);
        va_end(args);
    }

    /**
     * @brief Block execution until the Logger's TX buffer is empty
     *
     */
    inline void FLUSH(void)
    {
        Logger::loggerStream.flush();
    }
}  // namespace LOG

#else   // defined(VERBOSE)
namespace LOG
{
    inline void DEBUG(const Logger::Nodes::NodeBase&, const char*, ...) {}
    inline void INFO(const Logger::Nodes::NodeBase&, const char*, ...) {}
    inline void WARN(const Logger::Nodes::NodeBase&, const char*, ...) {}
    inline void ERROR(const Logger::Nodes::NodeBase&, const char*, ...) {}
    inline void FATAL(const Logger::Nodes::NodeBase&, const char*, ...) {}

    /**
     * @brief Block execution until the Logger's TX buffer is empty
     *
     */
    inline void FLUSH(void) {}
}  // namespace LOG
#endif  // defined(VERBOSE)

#endif  // __LOG_H__
