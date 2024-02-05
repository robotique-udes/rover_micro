#ifndef __ASSERT_HPP__
#define __ASSERT_HPP__

#include <assert.h>
#include "Arduino.h"

#ifndef __ASSERT_USE_STDERR
#define __ASSERT_USE_STDERR
#endif //__ASSERT_USE_STDERR

void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp)
{
    // transmit diagnostic informations through serial link.
    Serial.printf(FATAL, "ASSERT! (%s) condition isn't met.", __sexp);
    Serial.flush();
    // abort program execution.
    abort();
}

#endif // __ASSERT_HPP__
