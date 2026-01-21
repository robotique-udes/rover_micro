#ifndef BMSDATA_HPP
#define BMSDATA_HPP

#include "bms_configuration.hpp"

class bmsData
{
    public:

    private :
        int _amps;
        uint16_t _voltage = 0;
        uint16_t _loadVoltage = 0;
        uint16_t _chargeVoltage = 0;
        int _cellVoltage[BMS_CONFIG::BMS_TOTAL_CELLS] = {0};
        int _mcuTemp = 0;
        uint16_t statusFlags = 0;
        uint16_t faultFalgs = 0;
        int time = 0;
        uint32_t scriptChecksum = 0;
        uint16_t nodeAlive = 0;
        uint16_t capacityRuntime = 0;
        uint16_t stateOC = 0;
        uint16_t chargeDischargeCycle = 0;
        uint16_t bmsStateOC = 0;
        uint16_t bmsStatusFlags = 0;
        uint16_t bmsOpState = 0;
        uint16_t batteryStateOH = 0;

} 

#endif