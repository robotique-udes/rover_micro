#ifndef TIME_HPP
#define TIME_HPP

#if defined(__linux__)
#include <cstdint>

uint64_t millis(void);
uint64_t micros(void);
uint64_t nanos(void);

// Necessary because of the hardcoded paths in platformio...
#if defined(TEST_NATIVE)
#ifndef TIME_CPP
#include "time.cpp"
#endif  //  TIME_CPP
#endif  // defined (TEST_NATIVE)

#endif  // defined(__linux__)

#endif  // TIME_HPP
