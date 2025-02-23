#ifndef __LOG_H__
#define __LOG_H__

#if !defined(ARDUINO_ESP32S3_DEV)
#error CPU not supported
#endif

#if defined(VERBOSE)
#include "rover_lib2/helpers/macros.hpp"
#include <Stream.h>
#include <array>
#include <string_view>
#include <type_traits>

#if !defined(SEVERITY_LEVEL)
#define SEVERITY_LEVEL Logger::eSeverityLevels::DEBUG_
#endif

#if !defined(NODE_FILTER_SEVERITY_BYPASS)
#define NODE_FILTER_SEVERITY_BYPASS Logger::eSeverityLevels::INFO
#endif

#define DEFINE_LOG_NODE(name)                          \
    namespace Logger::Nodes                            \
    {                                                  \
        inline constexpr Logger::NodeBase name{#name}; \
    }

namespace Logger
{
    Stream& g_loggerOutput __attribute__((weak)) = Serial;

    struct NodeBase
    {
        constexpr NodeBase(const char* name): name(name) {}
        const char* const name;
    };

    template<typename... Nodes>
    struct NodeFilter
    {
        static constexpr bool isEnabled(const NodeBase& node_)
        {
            return ((std::string_view(node_.name) == std::string_view(Nodes::value)) || ...);
        }
    };

    enum class eSeverityLevels : size_t
    {
        DEBUG_ = 0,
        INFO,
        WARN,
        ERROR,
        FATAL,
        eLAST
    };
}  // namespace Logger

namespace
{
    const char* COLOR_RESET = "\033[97m";
    constexpr std::array<const char*, static_cast<size_t>(Logger::eSeverityLevels::eLAST)> SEVERITY_NAMES
        = {"DEBUG", "INFO", "WARN", "ERROR", "FATAL"};
    constexpr std::array<const char*, static_cast<size_t>(Logger::eSeverityLevels::eLAST)> SEVERITY_COLORS = {
        "\033[90m",  // DEBUG -> Gray
        "\033[97m",  // INFO -> White
        "\033[33m",  // WARN -> Yellow
        "\033[31m",  // ERROR -> Red
        "\033[31m"   // FATAL -> Red
    };

    constexpr const char* getSeverityColor(Logger::eSeverityLevels severity_)
    {
        return SEVERITY_COLORS[TO_UNDERLYING(severity_)];
    }

    constexpr const char* getSeverityName(Logger::eSeverityLevels severity_)
    {
        return SEVERITY_NAMES[TO_UNDERLYING(severity_)];
    }

    constexpr bool isSeverityEnabled(Logger::eSeverityLevels severity_)
    {
        return TO_UNDERLYING(severity_) >= TO_UNDERLYING(static_cast<decltype(severity_)>(SEVERITY_LEVEL));
    }

    constexpr bool isNodeEnabled(const Logger::NodeBase& node_, Logger::eSeverityLevels severity_)
    {
        if (TO_UNDERLYING(severity_) >= TO_UNDERLYING(NODE_FILTER_SEVERITY_BYPASS))
        {
            return true;
        }
        return EnabledNodes::isEnabled(node_);
    }

    template<Logger::eSeverityLevels SEVERITY>
    void logInternal(const Logger::NodeBase& node, const char* format, va_list args)
    {
        if (isSeverityEnabled(SEVERITY) && isNodeEnabled(node, SEVERITY))
        {
            Logger::g_loggerOutput.printf("%s[%lu][%s][%s]%s(%d): ",
                                          getSeverityColor(SEVERITY),
                                          millis(),
                                          getSeverityName(SEVERITY),
                                          node.name,
                                          __FILENAME__,
                                          __LINE__);

            Logger::g_loggerOutput.vprintf(format, args);
            Logger::g_loggerOutput.printf("\n%s", COLOR_RESET);
        }
    }
}  // namespace

namespace LOG
{
    inline void DEBUG(const Logger::NodeBase& node, const char* format, ...)
    {
        va_list args;
        va_start(args, format);
        logInternal<Logger::eSeverityLevels::DEBUG_>(node, format, args);
        va_end(args);
    }

    inline void INFO(const Logger::NodeBase& node, const char* format, ...)
    {
        va_list args;
        va_start(args, format);
        logInternal<Logger::eSeverityLevels::INFO>(node, format, args);
        va_end(args);
    }

    inline void WARN(const Logger::NodeBase& node, const char* format, ...)
    {
        va_list args;
        va_start(args, format);
        logInternal<Logger::eSeverityLevels::WARN>(node, format, args);
        va_end(args);
    }

    inline void ERROR(const Logger::NodeBase& node, const char* format, ...)
    {
        va_list args;
        va_start(args, format);
        logInternal<Logger::eSeverityLevels::ERROR>(node, format, args);
        va_end(args);
    }

    inline void FATAL(const Logger::NodeBase& node, const char* format, ...)
    {
        va_list args;
        va_start(args, format);
        logInternal<Logger::eSeverityLevels::FATAL>(node, format, args);
        va_end(args);
    }
}  // namespace LOG

#else   // defined(VERBOSE)
namespace LOG
{
    inline void DEBUG(const Logger::NodeBase&, const char*, ...) {}
    inline void INFO(const Logger::NodeBase&, const char*, ...) {}
    inline void WARN(const Logger::NodeBase&, const char*, ...) {}
    inline void ERROR(const Logger::NodeBase&, const char*, ...) {}
    inline void FATAL(const Logger::NodeBase&, const char*, ...) {}
}  // namespace LOG
#endif  // defined(VERBOSE)

#endif  // __LOG_H__
