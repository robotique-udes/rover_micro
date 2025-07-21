#include "gnss_parser.hpp"

namespace GNSSParser
{
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
                if (count < MAX_TOKENS)
                {
                    tokens_[count++] = &buffer_[i + 1];
                }
            }
        }
        return count;
    }

    inline float convertToDecimalDegrees(const char* nmeaCoord_, char direction_)
    {
        if (!nmeaCoord_ || strlen(nmeaCoord_) < 6)
        {
            LOG_WARN(Logger::Nodes::GNSS_PARSER, "Invalid NMEA Coord: %s", nmeaCoord_);
            return 0.0f;
        }

        float degMin = atof(nmeaCoord_);
        int degrees = static_cast<int>(degMin / 100.0f);
        float minutes = degMin - degrees * 100.0f;
        float decimal = degrees + minutes / 60.0f;

        if (direction_ == 'S' || direction_ == 'W')
        {
            decimal *= -1.0f;
        }

        return decimal;
    }

    /**
     * @brief
     *
     * @attention Assume rawSentence is a GGA message
     */
    bool parseGGA(char* rawSentence_, sGGAData& out_, Constants::UniHeadingQuality headingQuality_)
    {
        char* tokens[MAX_TOKENS] = {nullptr};
        size_t count = tokenize(rawSentence_, tokens);

        if (count >= 15)
        {
            if (tokens[0])
            {
                strncpy(out_.messageID, tokens[0], 6);
                out_.messageID[6] = '\0';
            }
            if (tokens[1])
            {
                float time = std::atof(tokens[1]);
                out_.utcTime.hours = static_cast<uint8_t>(time / 10000);
                out_.utcTime.minutes = static_cast<uint8_t>((static_cast<int>(time) % 10000) / 100);
                out_.utcTime.seconds = fmod(time, 100.0f);
            }
            if (tokens[2] && tokens[3])
            {
                out_.latitude = convertToDecimalDegrees(tokens[2], tokens[3][0]);
            }
            if (tokens[4] && tokens[5])
            {
                out_.longitude = convertToDecimalDegrees(tokens[4], tokens[5][0]);
            }
            if (tokens[6])
            {
                // Fix quality compared with uniheading for GPS vs GNSS
                uint8_t tempQuality = static_cast<uint8_t>(std::atoi(tokens[6]));

                if (tempQuality == 4)
                {
                    out_.fixQuality = Constants::GGAQuality::RTK;
                }
                else if (tempQuality == 1 && headingQuality_ != Constants::UniHeadingQuality::NO_HEADING)
                {
                    out_.fixQuality = Constants::GGAQuality::GNSS;
                }
                else if (tempQuality == 1)
                {
                    out_.fixQuality = Constants::GGAQuality::GPS;
                }
                else
                {
                    out_.fixQuality = Constants::GGAQuality::UNKNOWN;
                }
            }
            if (tokens[7])
            {
                out_.satellitesUsed = static_cast<uint8_t>(std::atoi(tokens[7]));
            }
            if (tokens[8])
            {
                out_.hdop = std::atof(tokens[8]);
            }
            if (tokens[9])
            {
                out_.mslAltitude = std::atof(tokens[9]);
            }
            if (tokens[10])
            {
                out_.altitudeUnits = tokens[10][0];
            }
            if (tokens[11])
            {
                out_.geoidSeparation = std::atof(tokens[11]);
            }
            if (tokens[12])
            {
                out_.geoidUnits = tokens[12][0];
            }
            if (tokens[13])
            {
                out_.ageOfDiffCorr = std::atof(tokens[13]);
            }
            if (tokens[14])
            {
                out_.diffRefStationID = static_cast<uint16_t>(std::atoi(tokens[14]));
            }
            if (tokens[15])
            {
                strncpy(out_.checksum, tokens[15], sizeof(out_.checksum) - 1);
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
    bool parseUniHeading(char* rawSentence_, sUniHeadingData& out_)
    {
        char* tokens[MAX_TOKENS] = {nullptr};
        size_t count = tokenize(rawSentence_, tokens);

        if (count > 10 && tokens[10])
        {
            if (strcmp(tokens[10], "L1_FLOAT") == 0)
            {
                out_.headingQuality = Constants::UniHeadingQuality::UNRELIABLE;
            }
            else if (strcmp(tokens[10], "L1_INT") == 0)
            {
                out_.headingQuality = Constants::UniHeadingQuality::RELIABLE;
            }
            else if (strcmp(tokens[10], "L1_FIXED") == 0)
            {
                out_.headingQuality = Constants::UniHeadingQuality::BEST;
            }
            else
            {
                out_.headingQuality = Constants::UniHeadingQuality::NO_HEADING;
            }
        }

        if (count > 12 && tokens[12])
        {
            out_.headingDeg = std::atof(tokens[12]);
            return true;
        }
        return false;
    }
}  // namespace GNSSParser