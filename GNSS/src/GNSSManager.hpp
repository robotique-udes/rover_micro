#ifndef GNSS_MANAGER_H
#define GNSS_MANAGER_H

#include <Arduino.h>
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/sensors/gnss_parser.hpp"
#include "rover_lib2/filters/circular_moving_average.hpp"


struct sGNSSData
{
    float latitude = 0.0f;
    float longitude = 0.0f;
    uint8_t fixQuality = 0;
    uint8_t satellites = 0;
    float headingDeg = 0.0f;

    inline bool hasValidFix() const { return fixQuality > 0; }
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
    static constexpr size_t MAX_SENTENCE_LENGTH = 100;
    static constexpr float RAD_TO_DEG_ = 57.295779513;
    static constexpr float DEG_TO_RAD_ = 0.017453293;

    Stream* _GNSSSerial;
    sGNSSData _currentData;
    CircularMovingAverage<CIRCULAR_WINDOW_SIZE> _headingFilter;

    char _sentenceBuffer[MAX_SENTENCE_LENGTH];
    size_t _buffer_Index;

public:
    explicit GNSSManager(Stream &serial_);
    void update(void);
    sGNSSData getData(void);
    float getFilteredHeading(void);

private:
    void parseMSG(char* buffer, size_t length_);
};

#endif // GNSS_MANAGER_H