#ifndef __MACRO_HPP__
#define __MACRO_HPP__

// Like a while(true) but better
#define EVER \
    ;        \
    ;

#define IN    // Specify pointer value is used inside function
#define OUT   // Specify pointer value will be overwritten inside function
#define INOUT // Specify pointer is first used and then overwritten

// Use this macro to removed unused warnings when compiling
#define REMOVE_WARN_UNUSED(fn) \
    {                          \
        auto garbage = fn;     \
    }

#define RCLC_RET_ON_ERR(fn)          \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            return false;            \
        }                            \
    }

template <typename T>
constexpr T MAP(T x, T in_min, T in_max, T out_min, T out_max)
{
    const T run = in_max - in_min;
    const T rise = out_max - out_min;
    const T delta = x - in_min;
    return (delta * rise) / run + out_min;
}

// #define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define GET_ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))
#define FOR_ALL(x) for (uint8_t i = 0; i < GET_ARRAY_SIZE(x); i++)

#define GET_WORST_OF(a, b) ((a == false || b == false) ? false : true)

#endif // __MACRO_HPP__
