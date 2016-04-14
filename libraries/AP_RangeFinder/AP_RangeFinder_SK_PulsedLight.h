// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_RANGEFINDER_SK_PULSEDLIGHT_H__
#define __AP_RANGEFINDER_SK_PULSEDLIGHT_H__

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

#include <AP_SerialManager/AP_SerialManager.h>

class AP_RangeFinder_SK_PulsedLight : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_SK_PulsedLight(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state);

    // static detection function
    static bool detect(RangeFinder &ranger, uint8_t instance);

    // update state
    void update(void);


private:
    static int8_t setAddr(uint8_t addr);
    static int8_t setRange(uint8_t range);
    static int8_t setStartPoint(uint8_t start);
    static int8_t setFreq(uint8_t freq);
    static int8_t startMeas(void);
    static int8_t setCmd(uint8_t* addrs, uint8_t* result, uint8_t param);
    static int8_t read_except(uint8_t* buf, uint8_t* expect, uint8_t expect_len);
    static uint8_t calcCheckSum(uint8_t* data, uint8_t len);
    bool isError(uint8_t* msg);
    uint16_t asscii2mm(uint8_t* msg);

    uint8_t _distance_msg[11];
    uint8_t _msg_pos;
    uint64_t _last_timestamp;
};
#endif  // __AP_RANGEFINDER_SK_PULSEDLIGHT_H__
