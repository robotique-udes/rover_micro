#ifndef GNSS_MANAGER_H
#define GNSS_MANAGER_H

#include <Arduino.h>
#include "rover_lib2/helpers/log.hpp"
#include <cstring>
#include <cstdint>


struct GNSSData 
{
    double latitude = 0.0;
    double longitude = 0.0;
    int fixQuality = 0;
    int satellites = 0;
    double headingDeg = 0.0;
    double rollDeg = 0.0;

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

public:
    explicit GNSSManager(Stream *serial_);
    void update(void);
    GNSSData getData(void) const;
    ~GNSSManager();

private:
    void parseNMEA(const std::array<char, MAX_SENTENCE_LENGTH>& sentence_, size_t length);
    void parseUNIHEADING(const std::array<char, MAX_SENTENCE_LENGTH>& sentence_, size_t length_);
    double convertToDecimalDegrees(const char* nmeaCoord_, char direction_);
};

#endif // GNSS_MANAGER_H