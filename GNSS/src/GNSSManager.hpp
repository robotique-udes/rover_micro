#ifndef GNSS_MANAGER_H
#define GNSS_MANAGER_H

#include <Arduino.h>
#include "rover_lib2/helpers/log.hpp"


struct GNSSData 
{
    double latitude = 0.0;
    double longitude = 0.0;
    bool validFix = false;
    int satellites = 0;
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

public:
    explicit GNSSManager(Stream *serial_);
    void update(void);
    GNSSData getData(void) const;
    ~GNSSManager();

private:
    void parseNMEA(const String& sentence_);
    double convertToDecimalDegrees(const String& nmeaCoord_, const char direction_);
};

#endif // GNSS_MANAGER_H