#ifndef MACROS_HPP
#define MACROS_HPP

#include <type_traits>

template<typename ENUM_T>
constexpr std::underlying_type_t<ENUM_T> TO_UNDERLYING(ENUM_T e) noexcept
{
    static_assert(std::is_enum_v<ENUM_T>, "TO_UNDERLYING() can only be used with enum types");
    return static_cast<std::underlying_type_t<ENUM_T>>(e);
}

#endif  // MACROS_HPP
