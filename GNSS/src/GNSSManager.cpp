#include "GNSSManager.hpp"

DEFINE_LOG_NODE(GNSS, Logger::eNodeState::OFF);

constexpr size_t maxLoopCount = 1'000UL;

bool sGNSSData::hasValidFix() const
{
    return fixQuality != Constants::eGGAQuality::UNKNOWN;
}

GNSSManager::GNSSManager(Stream& serial_):
    _GNSSSerial(serial_)
{
}

void GNSSManager::update(void)
{
    for (size_t i = 0UL; i < maxLoopCount && _GNSSSerial.available(); ++i)
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
            if (_bufferIndex > 0UL && _bufferIndex < _sentenceBuffer.size() - 1UL)
            {
                _sentenceBuffer[_bufferIndex] = '\0';
                LOG_DEBUG(Logger::Nodes::GNSS, "%s", _sentenceBuffer.data());

                if (_sentenceBuffer[0] == '$' || _sentenceBuffer[0] == '#')
                {
                    parseMSG(_sentenceBuffer, _bufferIndex);
                }
            }
            _bufferIndex = 0UL;
        }
        else if (ch != '\r' && _bufferIndex < MAX_SENTENCE_LENGTH - 1UL)
        {
            _sentenceBuffer[_bufferIndex++] = ch;
        }
    }
}

void GNSSManager::parseMSG(std::array<char, MAX_SENTENCE_LENGTH>& buffer_, size_t length_)
{
    if (length_ < 11UL || (buffer_[0] != '$' && buffer_[0] != '#'))
    {
        LOG_WARN(Logger::Nodes::GNSS,
                 "Le message reçu est tout cassé bozo: length: %zu, sentence: %.*s",
                 length_,
                 static_cast<int>(length_),
                 buffer_.data());
        return;
    }

    if (length_ < buffer_.size() && buffer_[length_] != '\0')
    {
        buffer_[length_] = '\0';
    }

    eGpsMsgType msgType = findGpsMsgType(buffer_, length_);

    switch (msgType)
    {
        case eGpsMsgType::GGA:
        {
            GNSSParser::sGGADataUsed GGA;
            if (GNSSParser::parseGGA(buffer_, GGA, _currentData.headingQuality))
            {
                _currentData.latitude = GGA.latitude;
                _currentData.longitude = GGA.longitude;
                _currentData.fixQuality = GGA.fixQuality;
                _currentData.satellites = GGA.satellitesUsed;
            }
            break;
        }

        case eGpsMsgType::UNI_HEADING:
        {
            GNSSParser::sUniHeadingDataUsed heading;
            if (GNSSParser::parseUniHeading(buffer_, heading))
            {
                _currentData.headingDeg = radToDeg(_headingFilter.addValue(degToRad(heading.headingDeg)));
                _currentData.headingQuality = heading.headingQuality;
            }
            break;
        }

        case eGpsMsgType::OTHER:
        {
            break;
        }

        default:
        {
            break;
        }
    }
}

eGpsMsgType GNSSManager::findGpsMsgType(std::array<char, MAX_SENTENCE_LENGTH>& buffer_, size_t length_)
{
    eGpsMsgType msgType = eGpsMsgType::OTHER;

    if (length_ < 11UL)
    {
        return msgType;
    }

    if ((buffer_[1] == 'G' && buffer_[2] == 'P' && buffer_[3] == 'G' && buffer_[4] == 'G' && buffer_[5] == 'A')
        || (buffer_[1] == 'G' && buffer_[2] == 'N' && buffer_[3] == 'G' && buffer_[4] == 'G' && buffer_[5] == 'A'))
    {
        msgType = eGpsMsgType::GGA;
    }

    if (buffer_[1] == 'U' && buffer_[2] == 'N' && buffer_[3] == 'I' && buffer_[4] == 'H' && buffer_[5] == 'E' && buffer_[6] == 'A'
        && buffer_[7] == 'D' && buffer_[8] == 'I' && buffer_[9] == 'N' && buffer_[10] == 'G' && buffer_[11] == 'A')
    {
        msgType = eGpsMsgType::UNI_HEADING;
    }

    return msgType;
}

sGNSSData GNSSManager::getData(void) const
{
    return _currentData;
}

float GNSSManager::getFilteredHeading(void) const
{
    return radToDeg(_headingFilter.getFilteredValue());
}
