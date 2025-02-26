#ifndef DEREF_ARRAY_HPP
#define DEREF_ARRAY_HPP

#include <cstddef>
#include <iterator>
#include <type_traits>

#include "rover_lib2/helpers/assert.hpp"

namespace RoverLib2
{
    template<typename TYPE, std::size_t NB_ELEM>
    class DerefArray
    {
      public:
        constexpr DerefArray() noexcept: _pData(nullptr) {}
        constexpr explicit DerefArray(TYPE& buffer) noexcept: _pData(&buffer) {}

        constexpr TYPE& at(std::size_t pos)
        {
            ASSERT(_pData != nullptr, "Data pointer is null");
            ASSERT(pos < NB_ELEM, "DerefArray::at: pos out of range");
            return _pData[pos];
        }

        constexpr const TYPE& at(std::size_t pos) const
        {
            ASSERT(_pData != nullptr, "Data pointer is null");
            ASSERT(pos < NB_ELEM, "DerefArray::at: pos out of range");
            return _pData[pos];
        }

        constexpr TYPE& operator[](std::size_t pos) noexcept
        {
            ASSERT(_pData != nullptr, "Data pointer is null");
            return _pData[pos];
        }

        constexpr const TYPE& operator[](std::size_t pos) const noexcept
        {
            ASSERT(_pData != nullptr, "Data pointer is null");
            return _pData[pos];
        }

        constexpr TYPE& front() noexcept
        {
            ASSERT(_pData != nullptr, "Data pointer is null");
            return _pData[0];
        }

        constexpr const TYPE& front() const noexcept
        {
            ASSERT(_pData != nullptr, "Data pointer is null");
            return _pData[0];
        }

        constexpr TYPE& back() noexcept
        {
            ASSERT(_pData != nullptr, "Data pointer is null");
            return _pData[NB_ELEM - 1];
        }

        constexpr const TYPE& back() const noexcept
        {
            ASSERT(_pData != nullptr, "Data pointer is null");
            return _pData[NB_ELEM - 1];
        }

        TYPE* data() noexcept
        {
            return _pData;
        }

        constexpr const TYPE* data() const noexcept
        {
            return _pData;
        }

        constexpr void set_data(TYPE* ptr) noexcept
        {
            _pData = ptr;
        }

        constexpr TYPE* begin() noexcept
        {
            ASSERT(_pData != nullptr, "Data pointer is null");
            return _pData;
        }

        constexpr const TYPE* begin() const noexcept
        {
            ASSERT(_pData != nullptr, "Data pointer is null");
            return _pData;
        }

        constexpr const TYPE* cbegin() const noexcept
        {
            ASSERT(_pData != nullptr, "Data pointer is null");
            return _pData;
        }

        constexpr TYPE* end() noexcept
        {
            ASSERT(_pData != nullptr, "Data pointer is null");
            return _pData + NB_ELEM;
        }

        constexpr const TYPE* end() const noexcept
        {
            ASSERT(_pData != nullptr, "Data pointer is null");
            return _pData + NB_ELEM;
        }

        constexpr const TYPE* cend() const noexcept
        {
            ASSERT(_pData != nullptr, "Data pointer is null");
            return _pData + NB_ELEM;
        }

        constexpr std::reverse_iterator<TYPE*> rbegin() noexcept
        {
            ASSERT(_pData != nullptr, "Data pointer is null");
            return std::reverse_iterator<TYPE*>(end());
        }

        constexpr std::reverse_iterator<const TYPE*> rbegin() const noexcept
        {
            ASSERT(_pData != nullptr, "Data pointer is null");
            return std::reverse_iterator<const TYPE*>(end());
        }

        constexpr std::reverse_iterator<const TYPE*> crbegin() const noexcept
        {
            ASSERT(_pData != nullptr, "Data pointer is null");
            return std::reverse_iterator<const TYPE*>(end());
        }

        constexpr std::reverse_iterator<TYPE*> rend() noexcept
        {
            ASSERT(_pData != nullptr, "Data pointer is null");
            return std::reverse_iterator<TYPE*>(begin());
        }

        constexpr std::reverse_iterator<const TYPE*> rend() const noexcept
        {
            ASSERT(_pData != nullptr, "Data pointer is null");
            return std::reverse_iterator<const TYPE*>(begin());
        }

        constexpr std::reverse_iterator<const TYPE*> crend() const noexcept
        {
            ASSERT(_pData != nullptr, "Data pointer is null");
            return std::reverse_iterator<const TYPE*>(begin());
        }

        constexpr bool empty() const noexcept
        {
            return NB_ELEM == 0;
        }

        constexpr std::size_t size() const noexcept
        {
            return NB_ELEM;
        }

        constexpr std::size_t max_size() const noexcept
        {
            return NB_ELEM;
        }

        constexpr void fill(const TYPE& value)
        {
            ASSERT(_pData != nullptr, "Data pointer is null");
            for (std::size_t i = 0; i < NB_ELEM; ++i)
            {
                _pData[i] = value;
            }
        }

        constexpr void swap(DerefArray& other) noexcept
        {
            std::swap(_pData, other._pData);
        }

        constexpr bool valid() const noexcept
        {
            return _pData != nullptr;
        }

      private:
        TYPE* _pData;
    };

    namespace DerefArray_
    {
        template<typename TYPE, std::size_t NB_ELEM>
        constexpr void swap(DerefArray<TYPE, NB_ELEM>& lhs, DerefArray<TYPE, NB_ELEM>& rhs) noexcept
        {
            lhs.swap(rhs);
        }
    }  // namespace DerefArray_

    template<typename TYPE, std::size_t NB_ELEM>
    constexpr bool operator==(const DerefArray<TYPE, NB_ELEM>& lhs, const DerefArray<TYPE, NB_ELEM>& rhs)
    {
        ASSERT(lhs.data() != nullptr && rhs.data() != nullptr, "Data pointer is null");

        for (std::size_t i = 0; i < NB_ELEM; ++i)
        {
            if (!(lhs[i] == rhs[i]))
            {
                return false;
            }
        }
        return true;
    }

    template<typename TYPE, std::size_t NB_ELEM>
    constexpr auto operator<=>(const DerefArray<TYPE, NB_ELEM>& lhs, const DerefArray<TYPE, NB_ELEM>& rhs)
    {
        ASSERT(lhs.data() != nullptr && rhs.data() != nullptr, "Data pointer is null");

        for (std::size_t i = 0; i < NB_ELEM; ++i)
        {
            if (auto cmp = lhs[i] <=> rhs[i]; cmp != 0)
            {
                return cmp;
            }
        }
        return std::strong_ordering::equal;
    }

}  // namespace RoverLib2

#endif  // DEREF_ARRAY_HPP
