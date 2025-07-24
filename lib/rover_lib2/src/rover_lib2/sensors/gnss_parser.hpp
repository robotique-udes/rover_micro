#ifndef ROVER_LIB2_SENSORS_GNSS_PARSER_HPP
#define ROVER_LIB2_SENSORS_GNSS_PARSER_HPP

#include <array>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/constants.hpp"

DEFINE_LOG_NODE(GNSS_PARSER, Logger::eNodeState::ON);

/**
 * @brief
 * To add a new GNSS message, add its data structure and its message parser
 * function
 *
 */
namespace GNSSParser
{
    constexpr size_t MAX_FIELDS = 16UL;
    constexpr size_t MAX_FIELD_LENGTH = 16UL;

    struct sUTCTime
    {
        uint8_t hours = 0U;
        uint8_t minutes = 0U;
        float seconds = 0.0F;
    };

    struct sGGAData
    {
        char messageID[7] = "";                                               // GGA protocol header
        sUTCTime utcTime;                                                     // UTC time (hhmmss.sss)
        float latitude = 0.0F;                                                // Latitude (ddmm.mmmm)
        char nsIndicator = 'N';                                               // N/S Indicator ('N' or 'S')
        float longitude = 0.0F;                                               // Longitude (dddmm.mmmm)
        char ewIndicator = 'E';                                               // E/W Indicator ('E' or 'W')
        Constants::eGGAQuality fixQuality = Constants::eGGAQuality::UNKNOWN;  // Position Fix Indicator
        uint8_t satellitesUsed = 0U;                                          // Number of satellites used
        float hdop = 0.0F;                                                    // Horizontal Dilution of Precision
        float mslAltitude = 0.0F;                                             // Mean Sea Level Altitude
        char altitudeUnits = 'M';                                             // Altitude units ('M' for meters)
        float geoidSeparation = 0.0F;                                         // Geoid separation
        char geoidUnits = 'M';                                                // Geoid separation units ('M' for meters)
        float ageOfDiffCorr = 0.0F;                                           // Age of differential corrections (seconds)
        uint16_t diffRefStationID = 0U;                                       // Differential reference station ID
        char checksum[3] = "";                                                // Checksum
    };

    struct sUniHeadingData  // Pas un message standard jsp c'est quoi les autres champs
    {
        float headingDeg = 0.0F;
        Constants::eUniHeadingQuality headingQuality = Constants::eUniHeadingQuality::NO_HEADING;
    };

    template<size_t SENTENCE_LENGTH>
    bool parseGGA(std::array<char, SENTENCE_LENGTH>& rawSentence_, sGGAData& out_, Constants::eUniHeadingQuality uhQuality_);
    template<size_t SENTENCE_LENGTH>
    bool parseUniHeading(std::array<char, SENTENCE_LENGTH>& rawSentence_, sUniHeadingData& out_);

    ///////////////////////////
    // Functions definitions //
    ///////////////////////////

    /**
     * @brief Find all the comma indices from a sentence
     *
     */
    template<size_t SENTENCE_LENGTH>
    inline size_t commaSegmenter(std::array<char, SENTENCE_LENGTH>& sentence_, std::array<size_t, MAX_FIELDS>& commaIndices_)
    {
        size_t count = 0UL;
        for (size_t i = 0UL; i < SENTENCE_LENGTH && sentence_[i] != '\0'; ++i)
        {
            if (sentence_[i] == ',')
            {
                if (count < MAX_FIELDS)
                {
                    commaIndices_[count++] = i;
                }
            }
        }

        // Add end of sentence as final index
        if (count < MAX_FIELDS)
        {
            commaIndices_[count++] = sentence_.size();
        }

        return count;
    }

    inline float convertToDecimalDegrees(const char* nmeaCoord_, char direction_)
    {
        if (!nmeaCoord_ || strlen(nmeaCoord_) < 6U)
        {
            LOG_WARN(Logger::Nodes::GNSS_PARSER, "Invalid NMEA Coord: %s", nmeaCoord_);
            return 0.0F;
        }

        float degMin = atof(nmeaCoord_);
        int degrees = static_cast<int>(degMin / 100.0F);
        float minutes = degMin - degrees * 100.0F;
        float decimal = degrees + minutes / 60.0F;

        if (direction_ == 'S' || direction_ == 'W')
        {
            decimal *= -1.0F;
        }

        return decimal;
    }

    template<size_t SENTENCE_LENGTH>
    bool getField(const std::array<char, SENTENCE_LENGTH>& sentence_,
                  const std::array<size_t, MAX_FIELDS>& commaIndices_,
                  std::array<char, MAX_FIELD_LENGTH>& field_,
                  size_t index_)
    {
        if (index_ >= MAX_FIELDS)
        {
            return false;
        }

        size_t start = (index_ == 0) ? 0 : commaIndices_[index_ - 1] + 1;
        size_t end = commaIndices_[index_];

        if (start >= end || end > SENTENCE_LENGTH)
        {
            return false;
        }

        size_t length = std::min(end - start, MAX_FIELD_LENGTH - 1);
        std::copy_n(&sentence_[start], length, field_.data());
        field_[length] = '\0';

        return true;
    }

    /**
     * @brief
     *
     * @attention Assume rawSentence is a GGA message
     */
    template<size_t SENTENCE_LENGTH>
    bool parseGGA(std::array<char, SENTENCE_LENGTH>& rawSentence_, sGGAData& out_, Constants::eUniHeadingQuality headingQuality_)
    {
        std::array<size_t, MAX_FIELDS> commaIndices{};
        size_t count = commaSegmenter(rawSentence_, commaIndices);

        if (count >= 15UL)
        {
            std::array<char, MAX_FIELD_LENGTH> field{};
            if (getField(rawSentence_, commaIndices, field, 0))
            {
                strncpy(out_.messageID, field.data(), 6U);
                out_.messageID[6] = '\0';
            }
            if (getField(rawSentence_, commaIndices, field, 1))
            {
                float time = std::atof(field.data());
                out_.utcTime.hours = static_cast<uint8_t>(time / 10000.0F);
                out_.utcTime.minutes = static_cast<uint8_t>((static_cast<int>(time) % 10000) / 100);
                out_.utcTime.seconds = fmod(time, 100.0F);
            }
            std::array<char, 2UL> secondField{}; // Only used to store 1 char (N/S, W/E)
            if (getField(rawSentence_, commaIndices, field, 2) && getField(rawSentence_, commaIndices, secondField, 3))
            {
                out_.latitude = convertToDecimalDegrees(field.data(), secondField[0]);
            }
            if (getField(rawSentence_, commaIndices, field, 4) && getField(rawSentence_, commaIndices, secondField, 5))
            {
                out_.longitude = convertToDecimalDegrees(field.data(), secondField[0]);
            }
            if (getField(rawSentence_, commaIndices, field, 6))
            {
                // Fix quality compared with uniheading for GPS vs GNSS
                uint8_t tempQuality = static_cast<uint8_t>(std::atoi(field.data()));

                if (tempQuality == 4U)
                {
                    out_.fixQuality = Constants::eGGAQuality::RTK;
                }
                else if (tempQuality == 1U && headingQuality_ != Constants::eUniHeadingQuality::NO_HEADING)
                {
                    out_.fixQuality = Constants::eGGAQuality::GNSS;
                }
                else if (tempQuality == 1U)
                {
                    out_.fixQuality = Constants::eGGAQuality::GPS;
                }
                else
                {
                    out_.fixQuality = Constants::eGGAQuality::UNKNOWN;
                }
            }
            if (getField(rawSentence_, commaIndices, field, 7))
            {
                out_.satellitesUsed = static_cast<uint8_t>(std::atoi(field.data()));
            }
            if (getField(rawSentence_, commaIndices, field, 8))
            {
                out_.hdop = std::atof(field.data());
            }
            if (getField(rawSentence_, commaIndices, field, 9))
            {
                out_.mslAltitude = std::atof(field.data());
            }
            if (getField(rawSentence_, commaIndices, field, 10))
            {
                out_.altitudeUnits = field[0];
            }
            if (getField(rawSentence_, commaIndices, field, 11))
            {
                out_.geoidSeparation = std::atof(field.data());
            }
            if (getField(rawSentence_, commaIndices, field, 12))
            {
                out_.geoidUnits = field[0];
            }
            if (getField(rawSentence_, commaIndices, field, 13))
            {
                out_.ageOfDiffCorr = std::atof(field.data());
            }
            if (getField(rawSentence_, commaIndices, field, 14))
            {
                out_.diffRefStationID = static_cast<uint16_t>(std::atoi(field.data()));
            }
            if (getField(rawSentence_, commaIndices, field, 15))
            {
                strncpy(out_.checksum, field.data(), sizeof(out_.checksum) - 1);
            }
            return true;
        }
        return false;
    }

    /**
     * @brief
     *
     * @attention Assume rawSentence is a UniHeading message
     */
    template<size_t SENTENCE_LENGTH>
    bool parseUniHeading(std::array<char, SENTENCE_LENGTH>& rawSentence_, sUniHeadingData& out_)
    {
        std::array<size_t, MAX_FIELDS> commaIndices{};
        size_t count = commaSegmenter(rawSentence_, commaIndices);

        if (count >= 12)
        {
            std::array<char, MAX_FIELD_LENGTH> field{};
            if (getField(rawSentence_, commaIndices, field, 10))
            {
                if (std::strncmp(field.data(), "L1_FLOAT", MAX_FIELD_LENGTH) == 0)
                {
                    out_.headingQuality = Constants::eUniHeadingQuality::UNRELIABLE;
                }
                else if (std::strncmp(field.data(), "L1_INT", MAX_FIELD_LENGTH) == 0)
                {
                    out_.headingQuality = Constants::eUniHeadingQuality::RELIABLE;
                }
                else if (std::strncmp(field.data(), "L1_FIXED", MAX_FIELD_LENGTH) == 0)
                {
                    out_.headingQuality = Constants::eUniHeadingQuality::BEST;
                }
                else
                {
                    out_.headingQuality = Constants::eUniHeadingQuality::NO_HEADING;
                }
            }

            if (getField(rawSentence_, commaIndices, field, 12))
            {
                out_.headingDeg = std::atof(field.data());
            }
            return true;
        }
        return false;
    }

};  // namespace GNSSParser

#endif  // ROVER_LIB2_SENSORS_GNSS_PARSER_HPP
