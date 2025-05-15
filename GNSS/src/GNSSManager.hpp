#ifndef GNSS_MANAGER_H
#define GNSS_MANAGER_H

#include <Arduino.h>
#include "rover_lib2/helpers/log.hpp"
#include <deque>


struct GNSSData 
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
    Stream* GNSSSerial;
    String buffer_;
    GNSSData currentData_;
    static constexpr size_t MAX_SENTENCE_LENGTH = 100;

    // Circular moving average buffer for heading
    static constexpr size_t HEADING_FILTER_WINDOW = 5;
    float headingBuffer_[HEADING_FILTER_WINDOW] = {0};
    size_t headingIndex_ = 0;
    size_t headingCount_ = 0;
    float sinSum_ = 0.0;
    float cosSum_ = 0.0;

public:
    explicit GNSSManager(Stream *serial_);
    void update(void);
    GNSSData getData(void) const;
    ~GNSSManager();

private:
    void parseMSG(const std::array<char, MAX_SENTENCE_LENGTH>& sentence_, size_t length);
    float convertToDecimalDegrees(const char* nmeaCoord_, char direction_);
    void updateHeadingFilter(float newHeadingDeg);
    float getFilteredHeading(void) const;
};

#endif // GNSS_MANAGER_H