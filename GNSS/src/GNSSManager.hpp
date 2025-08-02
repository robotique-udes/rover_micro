#ifndef GNSS_MANAGER_H
#define GNSS_MANAGER_H

#include <Arduino.h>
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/sensors/gnss_parser.hpp"
#include "rover_lib2/filters/circular_moving_average.hpp"

struct sGNSSData
{
    float latitude = 0.0F;
    float longitude = 0.0F;
    Constants::eGGAQuality fixQuality = Constants::eGGAQuality::NO_FIX;
    uint8_t satellites = 0U;
    float headingDeg = 0.0F;
    Constants::eHeadingQuality headingQuality = Constants::eHeadingQuality::NO_HEADING;

    bool hasValidFix() const;
};

enum class eGpsMsgType : uint8_t
{
    GGA,
    UNI_HEADING,
    OTHER,
};

/**
 * @brief
 * @warning Begin must be called on serial
 *
 */
class GNSSManager
{
  private:
    static constexpr size_t CIRCULAR_WINDOW_SIZE = 10UL;
    static constexpr size_t MAX_SENTENCE_LENGTH = 200UL;

    Stream& _GNSSSerial;
    sGNSSData _currentData;
    Filters::CircularMovingAverage<CIRCULAR_WINDOW_SIZE> _headingFilter;

    std::array<char, MAX_SENTENCE_LENGTH> _sentenceBuffer{};
    size_t _bufferIndex = 0;

  public:
    explicit GNSSManager(Stream& serial_);
    void update(void);
    sGNSSData getData(void) const;
    float getFilteredHeading(void) const;

  private:
    void parseMSG(std::array<char, MAX_SENTENCE_LENGTH>& buffer_, size_t length_);
    eGpsMsgType findGpsMsgType(std::array<char, MAX_SENTENCE_LENGTH>& buffer_, size_t length_);
};

#endif  // GNSS_MANAGER_H