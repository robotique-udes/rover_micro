// GNSSParser.hpp
#pragma once

#include <array>
#include <cstring>
#include <cstdlib>
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/macros.hpp"

DEFINE_LOG_NODE(GNSS_PARSER, Logger::eNodeState::OFF);

/**
 * @brief 
 * To add a new GNSS message, add its data structure and its message parser 
 * function (at the end)
 */
namespace GNSSParser 
{
    constexpr size_t MAX_TOKENS = 20;
    constexpr size_t MAX_FIELD_LENGTH = 20;

    // =======================
    // Structs for each message
    // =======================

    struct UTC_TIME 
    {
        uint8_t hours = 0;
        uint8_t minutes = 0;
        float seconds = 0.0f;
    };

    struct GGA_DATA 
    {
        char messageID[6] = "";         // GGA protocol header
        UTC_TIME utcTime;               // UTC time (hhmmss.sss)
        float latitude = 0.0f;          // Latitude (ddmm.mmmm)
        char nsIndicator = 'N';         // N/S Indicator ('N' or 'S')
        float longitude = 0.0f;         // Longitude (dddmm.mmmm)
        char ewIndicator = 'E';         // E/W Indicator ('E' or 'W')
        int fixQuality = 0;             // Position Fix Indicator
        int satellitesUsed = 0;         // Number of satellites used
        float hdop = 0.0f;              // Horizontal Dilution of Precision
        float mslAltitude = 0.0f;       // Mean Sea Level Altitude
        char altitudeUnits = 'M';       // Altitude units ('M' for meters)
        float geoidSeparation = 0.0f;   // Geoid separation
        char geoidUnits = 'M';          // Geoid separation units ('M' for meters)
        float ageOfDiffCorr = 0.0f;     // Age of differential corrections (seconds)
        int diffRefStationID = 0;       // Differential reference station ID
        char checksum[3] = "";          // Checksum
        bool valid = false;
    };

    struct UNI_HEADING_DATA      // Pas un message standard jsp c'est quoi les autres champs
    {
        float headingDeg = 0.0f;
        bool valid = false;
    };


    // =======================
    // Utility Functions
    // =======================

    // Get the message type as a fixed-length string view
    template<size_t SIZE>
    const char* getMsgType(const std::array<char, SIZE>& str_) 
    {
        static char type[15] = {0};
        size_t i = 0;
        while (i < SIZE && i < sizeof(type) - 1 && str_[i] != ',' && str_[i] != '\0') 
        {
            type[i] = str_[i];
            ++i;
        }
        type[i] = '\0';
        return type;
    }

    // Tokenize function: splits based on commas and fills fixed-size token array
    inline size_t tokenize(char* buffer_, char* tokens_[MAX_TOKENS]) 
    {
        size_t count = 0;
        tokens_[count++] = buffer_;
        for (size_t i = 0; buffer_[i] != '\0' && count < MAX_TOKENS; ++i) 
        {
            if (buffer_[i] == ',') 
            {
                buffer_[i] = '\0';
                if (i + 1 < MAX_TOKENS) 
                {
                    tokens_[count++] = &buffer_[i + 1];
                }
            }
        }
        return count;
    }


    // =======================
    // Conversion Function
    // =======================

    inline float convertToDecimalDegrees(const char* nmeaCoord_, char direction_) 
    {
        if (!nmeaCoord_ || strlen(nmeaCoord_) < 6)
        {
            LOG_ERROR(Logger::Nodes::GNSS_PARSER, "Invalid NMEA Coord: %s", nmeaCoord_);
            return 0.0f;
        }

        float degMin = atof(nmeaCoord_);
        int degrees = static_cast<int>(ROUND(degMin / 100.0f));
        float minutes = degMin - degrees * 100;
        float decimal = degrees + minutes / 60.0f;

        if (direction_ == 'S' || direction_ == 'W') 
        {
            decimal *= -1.0f;
            return decimal;
        }
        else
        {
            LOG_ERROR(Logger::Nodes::GNSS_PARSER, "Invalid direction: %c", direction_);
            return 0.0f;
        }
    }


    // =======================
    // Message Parsers
    // =======================

    inline void parseGGA(char* rawSentence_, GGA_DATA& out_) 
    {
        char* tokens[MAX_TOKENS] = {nullptr};
        size_t count = tokenize(rawSentence_, tokens);

        if (count >= 15) 
        {
            strncpy(out_.messageID, tokens[0], sizeof(out_.messageID) - 1);

            if (tokens[1]) 
            {
                float time = std::atof(tokens[1]);
                out_.utcTime.hours = static_cast<uint8_t>(time / 10000);
                out_.utcTime.minutes = static_cast<uint8_t>((static_cast<int>(time) % 10000) / 100);
                out_.utcTime.seconds = fmod(time, 100.0f);
            }
            out_.latitude = convertToDecimalDegrees(tokens[2], tokens[3][0]);
            out_.longitude = convertToDecimalDegrees(tokens[4], tokens[5][0]);
            out_.fixQuality = std::atoi(tokens[6]);
            out_.satellitesUsed = std::atoi(tokens[7]);
            out_.hdop = std::atof(tokens[8]);
            out_.mslAltitude = std::atof(tokens[9]);
            out_.altitudeUnits = tokens[10][0];
            out_.geoidSeparation = std::atof(tokens[11]);
            out_.geoidUnits = tokens[12][0];
            out_.ageOfDiffCorr = std::atof(tokens[13]);
            out_.diffRefStationID = std::atoi(tokens[14]);
            strncpy(out_.checksum, tokens[15], sizeof(out_.checksum) - 1);
            out_.valid = true;
        } 
        else 
        {
            out_.valid = false;
        }
    }


    inline void parseUniHeading(char* rawSentence_, UNI_HEADING_DATA& out_) 
    {
        char* tokens[MAX_TOKENS] = {nullptr};
        size_t count = tokenize(rawSentence_, tokens);

        if (count > 12 && tokens[12]) 
        {
            out_.headingDeg = std::atof(tokens[12]);
            out_.valid = true;
        }
        else
        {
            out_.valid = false;
        }
    }

} // namespace GNSSParser
