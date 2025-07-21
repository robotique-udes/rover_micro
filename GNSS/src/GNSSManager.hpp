#ifndef GNSS_MANAGER_H
#define GNSS_MANAGER_H

#include <Arduino.h>
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/sensors/gnss_parser.hpp"
#include "rover_lib2/filters/circular_moving_average.hpp"

struct sGNSSData
{
    float latitude = 0.0f;
    float longitude = 0.0f;
    Constants::GGAQuality fixQuality = Constants::GGAQuality::UNKNOWN;
    uint8_t satellites = 0;
    float headingDeg = 0.0f;
    Constants::UniHeadingQuality headingQuality = Constants::UniHeadingQuality::NO_HEADING;

    inline bool hasValidFix() const
    {
        return fixQuality != Constants::GGAQuality::UNKNOWN;
    }
};

/**
 * @brief
 * @attention Begin must be called on serial
 *
 */
class GNSSManager
{
  private:
    static constexpr size_t CIRCULAR_WINDOW_SIZE = 10;
    static constexpr size_t MAX_SENTENCE_LENGTH = 200;

    Stream& _GNSSSerial;
    sGNSSData _currentData;
    CircularMovingAverage<CIRCULAR_WINDOW_SIZE> _headingFilter;

    char _sentenceBuffer[MAX_SENTENCE_LENGTH] = {'\0'};
    size_t _buffer_Index;

  public:
    explicit GNSSManager(Stream& serial_);
    void update(void);
    sGNSSData getData(void);
    float getFilteredHeading(void);

  private:
    void parseMSG(char* buffer, size_t length_);
};

#endif  // GNSS_MANAGER_H