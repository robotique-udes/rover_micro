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
