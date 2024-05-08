#ifndef __MACROS_HPP__
#define __MACROS_HPP__

// =============================================================================
// macros.hpp defines some random helper macros that call be usefull
// =============================================================================

/// @example
///  void setup(void)
///  {
///     for(EVER)
///     {
///        /* Do something */
///     }
///  }
/// @brief Like a while(true) but better
#define EVER \
    ;        \
    ;

/// @example
///  void setup(void)
///  {
///      REMOVE_WARN_UNUSED(rcl_publish(&_pubLogger, &msg, NULL));
///  }
/// @brief Remove unused warnings when compiling
#define REMOVE_WARN_UNUSED(GARBAGE) \
    {                               \
        auto garbage = GARBAGE;     \
    }

/// @example
///  void setup(void)
///  {
///     RCLC_RET_ON_ERR(rclc_publisher_init_default(&_pubLogger,
///                                                 node_,
///                                                 ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
///                                                 NAME_LOG_TOPIC));
///  }
/// @brief Return false on rcl/rclc function error
#define RCLC_RET_ON_ERR(FUNCTION)     \
    {                                 \
        rcl_ret_t temp_rc = FUNCTION; \
        if ((temp_rc != RCL_RET_OK))  \
        {                             \
            return false;             \
        }                             \
    }

/// @example
///  void setup(void)
///  {
///      uint8_t arrayOfInt[10];
///      GET_ARRAY_SIZE(arrayOfInt); // = 10
///  }
/// @brief Return the size of an array
#define GET_ARRAY_SIZE(ARRAY) (sizeof(ARRAY) / sizeof(ARRAY[0]))

/// @example
///  void setup(void)
///  {
///      uint8_t arrayOfInt[10];
///      FOR_ALL(arrayOfInt)
///      {
///         arrayOfInt[i] = 0;
///      }
///  }
/// @brief Loop with 'i' iterator (ARRAY[i]) for all index of an array
#define FOR_ALL(ARRAY) for (uint8_t i = 0; i < GET_ARRAY_SIZE(ARRAY); i++)

#define COPY_ARRAY(src, dest, len)   \
    for (size_t i = 0; i < len; i++) \
    {                                \
        dest[i] = src[i];            \
    }

/// @example
///  void setup(void)
///  {
///     bool a = true;
///     bool b = false;
///     GET_WORST_OF(a, b) // Return False
///  }
///
///  Truth table:
///  |   | A | B | R |
///  | 0 | 0 | 0 | 0 |
///  | 1 | 0 | 1 | 0 |
///  | 2 | 1 | 0 | 0 |
///  | 3 | 1 | 1 | 1 |
///
/// @brief Return false if one of the arguments are false
#define GET_WORST_OF(a, b) ((a == false || b == false) ? false : true)

#define IN
#define OUT
#define INOUT

#define MAP(x, in_min, in_max, out_min, out_max) \
    (((float)(x) - (float)(in_min)) * ((float)(out_max) - (float)(out_min)) / ((float)(in_max) - (float)(in_min)) + (float)(out_min))
#endif // __MACROS_HPP__

#define IN_ERROR(VAR, ERROR, GOAL) ((abs(VAR) < (abs(GOAL) + ERROR) && abs(VAR) > (abs(GOAL) - ERROR)))
