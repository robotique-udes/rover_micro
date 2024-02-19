#ifndef __ASSERT_HPP__
#define __ASSERT_HPP__

#if defined(VERBOSE)
#include "helpers/log.hpp"

#define GET_MACRO_ASSERT(_0, _1, _2, ASSERT, ...) ASSERT
#define ASSERT(...) GET_MACRO_ASSERT(_0, ##__VA_ARGS__, ASSERT2, ASSERT1, ASSERT0)(__VA_ARGS__)

#define ASSERT2(condition, ...)           \
    {                                     \
        if (condition)                    \
        {                                 \
            LOG(FATAL, __VA_ARGS__);      \
            Serial.flush();               \
            abort();                      \
        }                                 \
    }

#define ASSERT1(condition)                \
    {                                     \
        if (condition)                    \
        {                                 \
            LOG(FATAL, "%s", #condition); \
            Serial.flush();               \
            abort();                      \
        }                                 \
    }

#define ASSERT0() \
    {             \
        abort();  \
    }

#else // defined(VERBOSE)
#define ASSERT(condition) ? abort() : (void)0;
#endif // defined(VERBOSE)

#endif // __ASSERT_HPP__
