#ifndef MORSE_PLAYER_HPP
#define MORSE_PLAYER_HPP

#include <cstdint>
#include <cstddef>
#include "rover_lib2/helpers/time.hpp"

class MorsePlayer
{
  public:
    // PARIS standard: dit_ms = 1200 / WPM. At 18 WPM -> 66.7ms
    static constexpr float DIT_MS = 66.7F;
    static constexpr float DAH_MS = DIT_MS * 3.0F;
    static constexpr float SYMBOL_GAP_MS = DIT_MS;
    static constexpr float CHAR_GAP_MS = DIT_MS * 3.0F;
    static constexpr float WORD_GAP_MS = DIT_MS * 7.0F;

    enum class eState : uint8_t
    {
        IDLE,
        SYMBOL_ON,
        SYMBOL_GAP,
        CHAR_GAP,
        WORD_GAP,
        DONE
    };

    void start(const uint8_t* buffer_, uint8_t length_)
    {
        _buffer = buffer_;
        _length = length_;
        _charIndex = 0;
        _currentSymbols = nullptr;
        _symbolIndex = 0;
        _actuatorOn = false;
        enterState(eState::CHAR_GAP, static_cast<uint64_t>(CHAR_GAP_MS));  // small lead-in
    }

    void update()
    {
        if ((Time::millis() - _stateStart) < _stateDuration)
        {
            return;
        }
        advance();
    }

    bool isActuatorOn() const
    {
        return _actuatorOn;
    }
    bool isBusy() const
    {
        return _state != eState::IDLE && _state != eState::DONE;
    }

  private:
    static const char* symbolsFor(uint8_t character_)
    {
        char c = static_cast<char>(character_);

        if (c >= 'a' && c <= 'z')
        {
            c = static_cast<char>(c - 'a' + 'A');
        }

        if (c >= 'A' && c <= 'Z')
        {
            return LETTER_TABLE[c - 'A'];
        }
        else if (c >= '0' && c <= '9')
        {
            return DIGIT_TABLE[c - '0'];
        }
        else if (c == '-')
        {
            return "-";
        }
        else if (c == '.')
        {
            return ".";
        }
        return nullptr;
    }

    void advance()
    {
        switch (_state)
        {
            case eState::CHAR_GAP:
                [[fallthrough]];
            case eState::WORD_GAP:
                beginNextCharacter();
                break;

            case eState::SYMBOL_ON:
                enterState(eState::SYMBOL_GAP, static_cast<uint64_t>(SYMBOL_GAP_MS));
                _actuatorOn = false;
                break;

            case eState::SYMBOL_GAP:
                ++_symbolIndex;
                playNextSymbolOrEndChar();
                break;

            case eState::DONE:
                [[fallthrough]];
            case eState::IDLE:
                [[fallthrough]];
            default:
                break;
        }
    }

    void beginNextCharacter()
    {
        if (_charIndex >= _length)
        {
            _state = eState::DONE;
            _actuatorOn = false;
            return;
        }

        const uint8_t character = _buffer[_charIndex];
        ++_charIndex;

        if (character == static_cast<uint8_t>(' '))
        {
            enterState(eState::WORD_GAP, static_cast<uint64_t>(WORD_GAP_MS));
            _actuatorOn = false;
            return;
        }

        _currentSymbols = symbolsFor(character);
        _symbolIndex = 0;

        if (_currentSymbols == nullptr)
        {
            enterState(eState::CHAR_GAP, static_cast<uint64_t>(CHAR_GAP_MS));
            _actuatorOn = false;
            return;
        }

        playSymbol();
    }

    void playNextSymbolOrEndChar()
    {
        if (_currentSymbols[_symbolIndex] == '\0')
        {
            enterState(eState::CHAR_GAP, static_cast<uint64_t>(CHAR_GAP_MS));
            _actuatorOn = false;
            return;
        }
        playSymbol();
    }

    void playSymbol()
    {
        const float durationMs = (_currentSymbols[_symbolIndex] == '-') ? DAH_MS : DIT_MS;
        enterState(eState::SYMBOL_ON, static_cast<uint64_t>(durationMs));
        _actuatorOn = true;
    }

    void enterState(eState state_, uint64_t durationMs_)
    {
        _state = state_;
        _stateStart = Time::millis();
        _stateDuration = durationMs_;
    }

    static constexpr const char* LETTER_TABLE[26]
        = {".-", "-...", "-.-.", "-..",  ".",   "..-.", "--.", "....", "..",   ".---", "-.-",  ".-..", "--",
           "-.", "---",  ".--.", "--.-", ".-.", "...",  "-",   "..-",  "...-", ".--",  "-..-", "-.--", "--.."};

    static constexpr const char* DIGIT_TABLE[10]
        = {"-----", ".----", "..---", "...--", "....-", ".....", "-....", "--...", "---..", "----."};

    const uint8_t* _buffer = nullptr;
    uint8_t _length = 0;
    uint8_t _charIndex = 0;
    const char* _currentSymbols = nullptr;
    uint8_t _symbolIndex = 0;

    eState _state = eState::IDLE;
    uint64_t _stateStart = 0;
    uint64_t _stateDuration = 0;
    bool _actuatorOn = false;
};

#endif  // MORSE_PLAYER_HPP