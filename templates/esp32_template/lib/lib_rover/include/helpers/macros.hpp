#ifndef __MACROS_HPP__
#define __MACROS_HPP__

// =============================================================================
// macros.hpp defines some random helper macros that call be usefull
// =============================================================================

// Like a while(true) but better
#define EVER \
    ;        \
    ;

// Removes unused warnings when compiling
#define REMOVE_WARN_UNUSED(GARBAGE) \
    {                               \
        auto garbage = GARBAGE;     \
    }

// Return false on rcl/rclc function error
#define RCLC_RET_ON_ERR(FUNCTION)     \
    {                                 \
        rcl_ret_t temp_rc = FUNCTION; \
        if ((temp_rc != RCL_RET_OK))  \
        {                             \
            return false;             \
        }                             \
    }

// Return the size of an array
#define GET_ARRAY_SIZE(ARRAY) (sizeof(ARRAY) / sizeof(ARRAY[0]))

// Loop with 'i' iterator (ARRAY[i]) for all index of an array
#define FOR_ALL(ARRAY) for (uint8_t i = 0; i < GET_ARRAY_SIZE(ARRAY); i++)

// Return false if one of the arguments are false
#define GET_WORST_OF(a, b) ((a == false || b == false) ? false : true)

#endif // __MACROS_HPP__
