#include "GNSSManager.hpp"

DEFINE_LOG_NODE(GNSS, Logger::eNodeState::OFF);

constexpr size_t maxLoopCount = 1000UL;

GNSSManager::GNSSManager(Stream& serial_):
    _GNSSSerial(serial_),
    _buffer_Index(0)
{
}

void GNSSManager::update(void)
{
    for (size_t i = 0; i < maxLoopCount && _GNSSSerial.available(); ++i)
    {
        int c = _GNSSSerial.read();
        if (c == -1)
        {
            continue;
        }
        char ch = static_cast<char>(c);
        // LOG_DEBUG(Logger::Nodes::GNSS, "Charactere recu: %c", ch);

        if (ch == '\n' || ch == '\r')
        {
            if (_buffer_Index > 0UL && _buffer_Index < MAX_SENTENCE_LENGTH - 1UL)
            {
                _sentenceBuffer[_buffer_Index] = '\0';
                LOG_DEBUG(Logger::Nodes::GNSS, "%s", _sentenceBuffer);

                if (_sentenceBuffer[0] == '$' || _sentenceBuffer[0] == '#')
                {
                    parseMSG(_sentenceBuffer, _buffer_Index);
                }
            }
            _buffer_Index = 0;
        }
        else if (ch != '\r' && _buffer_Index < MAX_SENTENCE_LENGTH - 1UL)
        {
            _sentenceBuffer[_buffer_Index++] = ch;
        }
    }
}

void GNSSManager::parseMSG(char* buffer_, size_t length_)
{
    if (length_ < 10 || (buffer_[0] != '$' && buffer_[0] != '#'))
    {
        LOG_WARN(Logger::Nodes::GNSS, "Le message reçu est tout cassé bozo: lenght: %d, sentence 0: %s", length_, buffer_);
        return;
    }

    bool isGGA = (buffer_[1] == 'G' && buffer_[2] == 'P' && buffer_[3] == 'G' && buffer_[4] == 'G' && buffer_[5] == 'A')
                 || (buffer_[1] == 'G' && buffer_[2] == 'N' && buffer_[3] == 'G' && buffer_[4] == 'G' && buffer_[5] == 'A');

    bool isUniHeading
        = (buffer_[1] == 'U' && buffer_[2] == 'N' && buffer_[3] == 'I' && buffer_[4] == 'H' && buffer_[5] == 'E'
           && buffer_[6] == 'A' && buffer_[7] == 'D' && buffer_[8] == 'I' && buffer_[9] == 'N' && buffer_[10] == 'G');

    if (isGGA)
    {
        GNSSParser::sGGAData GGA;
        if (GNSSParser::parseGGA(buffer_, GGA, _currentData.headingQuality))
        {
            _currentData.latitude = GGA.latitude;
            _currentData.longitude = GGA.longitude;
            _currentData.fixQuality = GGA.fixQuality;
            _currentData.satellites = GGA.satellitesUsed;
        }
    }
    else if (isUniHeading)
    {
        GNSSParser::sUniHeadingData heading;
        if (GNSSParser::parseUniHeading(buffer_, heading))
        {
            _currentData.headingDeg = RAD_TO_DEG_ * _headingFilter.addValue(heading.headingDeg * DEG_TO_RAD_);
            _currentData.headingQuality = heading.headingQuality;
        }
    }
}

sGNSSData GNSSManager::getData(void)
{
    return _currentData;
}

float GNSSManager::getFilteredHeading(void)
{
    return _headingFilter.getAverage() * RAD_TO_DEG_;
}
