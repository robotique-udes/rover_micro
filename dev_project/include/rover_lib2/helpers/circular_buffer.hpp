#ifndef CIRCULAR_BUFFER_HPP
#define CIRCULAR_BUFFER_HPP

#include <array>
#include <cstddef>
#include <optional>

#include "rover_lib2/helpers/log.hpp"

DEFINE_LOG_NODE(CircularBuffer, Logger::eNodeState::ON);

template<typename TYPE, std::size_t SIZE>
class CircularBuffer
{
  public:
    enum class eErrorCode
    {
        SUCCESS,
        SUCCESS_DATA_LOSS,
        ERROR
    };

    CircularBuffer()
    {
        this->init();
    }

    /**
     * @brief Adds a new value to the circular buffer
     *
     * @param value Takes reference to value but value is copied, no need to 
     * keep it alive
     * @return eErrorCode::SUCCESS on success, eErrorCode::SUCCESS_DATA_LOSS
     * buffer is full and the data was added by overwriting unread data.
     */
    eErrorCode addValue(const TYPE& value_)
    {
        _buffer[_addCursor] = value_;
        _addCursor = (_addCursor + 1UL) % _buffer.size();

        if (_size < _buffer.size())
        {
            _size++;
            return eErrorCode::SUCCESS;
        }
        else
        {
            return eErrorCode::SUCCESS_DATA_LOSS;
        }
    }

    std::optional<TYPE> getValue(void)
    {
        if (_size == 0)
        {
            return std::nullopt;
        }

        std::size_t getCursor = (_addCursor - _size + _buffer.size()) % _buffer.size();
        std::optional<TYPE> value = _buffer[getCursor];

        _buffer[getCursor] = std::nullopt;
        _size--;

        return value;
    }

    void emptyBuffer(void)
    {
        for (size_t i = 0UL; i < _buffer.size(); i++)
        {
            if (!this->getValue())
            {
                return;
            }
        }

        this->init();
    }

    std::size_t size(void) const
    {
        return _size;
    }

  private:
    void init(void)
    {
        _addCursor = 0UL;
        _size = 0UL;
    }

    std::array<std::optional<TYPE>, SIZE> _buffer;
    std::size_t _addCursor = 0UL;
    std::size_t _size = 0UL;
};

#endif  // CIRCULAR_BUFFER_HPP
