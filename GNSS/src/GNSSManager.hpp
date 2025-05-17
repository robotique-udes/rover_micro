#ifndef GNSS_MANAGER_H
#define GNSS_MANAGER_H

#include <Arduino.h>
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/sensors/gnss_parser.hpp"
#include "rover_lib2/filters/circular_moving_average.hpp"
#include <deque>


struct GNSS_DATA
{
    float latitude = 0.0;
    float longitude = 0.0;
    int fixQuality = 0;
    int satellites = 0;
    float headingDeg = 0.0;

    bool hasValidFix() const { return fixQuality > 0; }
};


/**
 * @brief 
 * @attention Begin must be called on serial 
 * 
 */
class GNSSManager 
{
private:
    Stream* _GNSSSerial;
    String _buffer;
    GNSS_DATA _sCurrentData;
    static constexpr size_t MAX_SENTENCE_LENGTH = 100;

    CircularMovingAverage<10> _headingFilter;  // Circular window size of 10

public:
    explicit GNSSManager(Stream *serial_);
    void update(void);
    GNSS_DATA getData(void) const;
    void updateHeadingFilter(float headingDeg_);
    float getFilteredHeading(void) const;
    ~GNSSManager();

private:
    void parseMSG(const std::array<char, MAX_SENTENCE_LENGTH>& sentence_, size_t length_);
    float convertToDecimalDegrees(const char* nmeaCoord_, char direction_);
};

#endif // GNSS_MANAGER_H