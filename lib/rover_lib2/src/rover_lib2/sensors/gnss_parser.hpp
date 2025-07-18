#ifndef ROVER_LIB2_SENSORS_GNSS_PARSER_HPP
#define ROVER_LIB2_SENSORS_GNSS_PARSER_HPP

#include <array>
#include <cstring>
#include <cstdlib>
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/macros.hpp"

DEFINE_LOG_NODE(GNSS_PARSER, Logger::eNodeState::ON);

/**
 * @brief
 * To add a new GNSS message, add its data structure and its message parser
 * function
 *
 */
namespace GNSSParser
{
    constexpr size_t MAX_TOKENS = 20;
    constexpr size_t MAX_FIELD_LENGTH = 20;

    struct sUTCTime
    {
        uint8_t hours = 0;
        uint8_t minutes = 0;
        float seconds = 0.0f;
    };

    struct sGGAData
    {
        char messageID[7] = "";         // GGA protocol header
        sUTCTime utcTime;               // UTC time (hhmmss.sss)
        float latitude = 0.0f;          // Latitude (ddmm.mmmm)
        char nsIndicator = 'N';         // N/S Indicator ('N' or 'S')
        float longitude = 0.0f;         // Longitude (dddmm.mmmm)
        char ewIndicator = 'E';         // E/W Indicator ('E' or 'W')
        uint8_t fixQuality = 0;         // Position Fix Indicator
        uint8_t satellitesUsed = 0;     // Number of satellites used
        float hdop = 0.0f;              // Horizontal Dilution of Precision
        float mslAltitude = 0.0f;       // Mean Sea Level Altitude
        char altitudeUnits = 'M';       // Altitude units ('M' for meters)
        float geoidSeparation = 0.0f;   // Geoid separation
        char geoidUnits = 'M';          // Geoid separation units ('M' for meters)
        float ageOfDiffCorr = 0.0f;     // Age of differential corrections (seconds)
        uint16_t diffRefStationID = 0;  // Differential reference station ID
        char checksum[3] = "";          // Checksum
    };

    struct sUniHeadingData  // Pas un message standard jsp c'est quoi les autres champs
    {
        float headingDeg = 0.0f;
    };

    bool parseGGA(char* rawSentence_, sGGAData& out_);
    bool parseUniHeading(char* rawSentence_, sUniHeadingData& out_);
};  // namespace GNSSParser

#endif  // ROVER_LIB2_SENSORS_GNSS_PARSER_HPP
