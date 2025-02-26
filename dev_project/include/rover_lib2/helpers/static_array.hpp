#ifndef STATIC_ARRAY_HPP
#define STATIC_ARRAY_HPP

#include <array>
#include <algorithm>
#include <utility>
#include <cstddef>
#include <type_traits>

template<typename TYPE, std::size_t NB_ELEM>
class StaticArray
{
  public:
    constexpr StaticArray(const std::array<TYPE, NB_ELEM>& data): _data(data) {}

    template<typename... Args, typename = std::enable_if_t<sizeof...(Args) == NB_ELEM && (std::is_convertible_v<Args, TYPE> && ...)>>
    constexpr StaticArray(Args&&... args): _data{std::forward<Args>(args)...}
    {
    }

    constexpr bool contains(const TYPE& value) const
    {
        for (std::size_t i = 0; i < NB_ELEM; ++i)
        {
            if (_data[i] == value)
            {
                return true;
            }
        }
        return false;
    }

    constexpr const TYPE& operator[](std::size_t index) const
    {
        return _data[index];
    }

    constexpr std::size_t size() const
    {
        return NB_ELEM;
    }

    constexpr const std::array<TYPE, NB_ELEM>& data() const
    {
        return _data;
    }

    constexpr auto begin() const
    {
        return _data.begin();
    }

    constexpr auto end() const
    {
        return _data.end();
    }

  private:
    std::array<TYPE, NB_ELEM> _data;
};

#endif  // STATIC_ARRAY_HPP
