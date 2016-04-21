// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "AP_RangeFinder_SK_PulsedLight.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

static AP_HAL::UARTDriver* _port = NULL;

#define RFINDER_NODATA_TIMEOUT 2000000 // us

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_SK_PulsedLight::AP_RangeFinder_SK_PulsedLight(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(_ranger, instance, _state),
    _msg_pos(0),
    _last_timestamp(0)
{
    memset(_distance_msg, 0, sizeof(_distance_msg));

    startMeas();
}

/* 
   detect if a PulsedLight rangefinder is connected. 
*/
bool AP_RangeFinder_SK_PulsedLight::detect(RangeFinder &_ranger, uint8_t instance)
{
    if (_ranger._serial_manager == NULL){
        return false;
    }

    _port = _ranger._serial_manager->find_serial(AP_SerialManager::SerialProtocol_SK_PulseLight, 0);
    if (_port == NULL) {
      return false;
    }

    _port->begin(AP_SERIALMANAGER_SK_PULSELIGHT_BAUD,
      AP_SERIALMANAGER_SK_PULSELIGHT_BUFSIZE_RX,
      AP_SERIALMANAGER_SK_PULSELIGHT_BUFSIZE_TX);

    if (setAddr(0x80) != 0){
        return false;
    }

    int16_t val_info_range_max = _ranger._max_distance_cm[instance] / 100;
    int16_t range = 5; // m  5,10,30,50,80

    if (val_info_range_max <= 5){
    }else if (val_info_range_max <= 10){
        range = 10;
    }else if (val_info_range_max <= 30){
        range = 30;
    }else if (val_info_range_max <= 50){
        range = 50;
    }else{
        range = 80;
    }

    if (setRange(range) != 0){
        return false;
    }

    if (setStartPoint(1) != 0){
        return false;
    }

    if (setFreq(10) != 0){
        return false;
    }

    return true;
}

/* 
   update the state of the sensor
*/
void AP_RangeFinder_SK_PulsedLight::update(void)
{
    uint8_t header[3] = {0x80, 0x06, 0x83};
    int16_t loop_cnt;
    uint8_t checksum;
    uint8_t checksum_index;
    int16_t available = _port->available();
    uint8_t count = 0;
    float sum = 0;

    if (available <= 0){
        return;
    }

    // read header
    while (available > 0 && _msg_pos < sizeof(header)){
        switch(_msg_pos){
            case 0:
            {
                if (_port->read() == header[_msg_pos]){
                    _distance_msg[_msg_pos++] = header[0];
                }

                available--;
            }
            break;
            case 1:
            {
                if (_port->read() == header[_msg_pos]){
                    _distance_msg[_msg_pos++] = header[1];
                }

                available--;
            }
            break;
            case 2:
            {
                if (_port->read() == header[_msg_pos]){
                    _distance_msg[_msg_pos++] = header[2];
                }

                available--;
            }
            break;
        }
    }

    do{
        if (available <= 0 || _msg_pos < sizeof(header)){
            // still need wait a valid header
            break;
        }

        // read 7 bytes payload and 1 byte checksum
        loop_cnt = 8;

        if (available < loop_cnt){
            loop_cnt = available;
        }

        for (int16_t i = 0; i < loop_cnt; i++){
            _distance_msg[_msg_pos++] = _port->read();
            available--;
        }

        if (_msg_pos != sizeof(_distance_msg)){ // not a valid msg at all
            break;
        }

        _msg_pos = 0; // to restart receive

        // check valid
        checksum_index = sizeof(_distance_msg) - 1;
        checksum = calcCheckSum(_distance_msg, checksum_index);

        if (_distance_msg[checksum_index] != checksum){
            break;
        }

        // unpack msg
        if (isError(_distance_msg)){
            break;
        }

        // tran ascii distance to cm
        sum += (float)asscii2mm(_distance_msg);
        count++;

        _last_timestamp = hal.scheduler->micros64();
    }while(available > 0);

    if (count > 0){
        state.distance_cm = (uint16_t)(sum / ((float)count * 10.0f));
        state.distance_cm += ranger._offset[state.instance];

        update_status();
    }

    if (hal.scheduler->micros64() - _last_timestamp >= RFINDER_NODATA_TIMEOUT) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

bool AP_RangeFinder_SK_PulsedLight::isError(uint8_t* msg){
    return (msg[3] == 'E' && msg[4] == 'R' && msg[5] == 'R')
         || msg[3] < 0x30
         || msg[4] < 0x30
         || msg[5] < 0x30
         || msg[6] != '.'
         || msg[7] < 0x30
         || msg[8] < 0x30
         || msg[9] < 0x30;
}

uint16_t AP_RangeFinder_SK_PulsedLight::asscii2mm(uint8_t* msg){
    uint16_t mm;

    mm = (msg[3] - 0x30) * 100000;
    mm += (msg[4] - 0x30) * 10000;
    mm += (msg[5] - 0x30) * 1000;
    mm += (msg[7] - 0x30) * 100;
    mm += (msg[8] - 0x30) * 10;
    mm += (msg[9] - 0x30);

    return mm;
}

int8_t AP_RangeFinder_SK_PulsedLight::setAddr(uint8_t addr){
    uint8_t addrs[5] = {0xFA, 0x04, 0x01, 0x00, 0x00};
    uint8_t result[4] = {0xFA, 0x4, 0x81, 0x81};

    return setCmd5Result4(addrs, result, addr);
}

int8_t AP_RangeFinder_SK_PulsedLight::setRange(uint8_t range){
    uint8_t addrs[5] = {0xFA, 0x04, 0x09, 0x00, 0x00};
    uint8_t result[4] = {0xFA, 0x4, 0x89, 0x79};

    return setCmd5Result4(addrs, result, range);
}

// - 0 start from tail
// - 1 start from front
int8_t AP_RangeFinder_SK_PulsedLight::setStartPoint(uint8_t start){
    uint8_t addrs[5] = {0xFA, 0x04, 0x08, 0x00, 0x00};
    uint8_t result[4] = {0xFA, 0x4, 0x88, 0x7A};

    return setCmd5Result4(addrs, result, start);
}

int8_t AP_RangeFinder_SK_PulsedLight::setFreq(uint8_t freq){
    uint8_t addrs[5] = {0xFA, 0x04, 0x0A, 0x00, 0x00};
    uint8_t result[4] = {0xFA, 0x4, 0x8A, 0x78};

    return setCmd5Result4(addrs, result, freq);
}

int8_t AP_RangeFinder_SK_PulsedLight::startMeas(void){
    uint8_t addrs[5] = {0x80, 0x06, 0x03, 0x00};

    addrs[3] = calcCheckSum(addrs, sizeof(addrs) - 1);

    _port->write((const uint8_t *)addrs, sizeof(addrs));

    return 0;
}

int8_t AP_RangeFinder_SK_PulsedLight::setCmd5Result4(uint8_t* addrs, uint8_t* result, uint8_t param){

    uint8_t temp[4];

    addrs[3] = param;

    addrs[4] = calcCheckSum(addrs, 4);

    _port->write((const uint8_t *)addrs, 5);

    // wait for result, 100ms timeout
    uint64_t start =  hal.scheduler->micros64();
    uint8_t pos = 0;
    int8_t ret = 0;

    while(hal.scheduler->micros64() - start < 100000){

        ret = read_except(&temp[pos], &result[pos], 4 - pos);
        if (ret < 0){
            pos = 0;
        } else{
            pos += ret;
        }

        if (pos == sizeof(result)){
            return 0;
        }
    }

    return -1;
}

// - -1 need resync
// - n sync data
int8_t AP_RangeFinder_SK_PulsedLight::read_except(uint8_t* buf,
    uint8_t* expect, uint8_t expect_len){
    int16_t available = _port->available();
    uint8_t loop_cnt = expect_len;

    if (available <= expect_len){
        loop_cnt = available;
    }


    for (uint8_t i = 0; i < loop_cnt; i++){
        if (_port->read() != expect[i]){
            return -(i + 1);
        }

        buf[i] = expect[i];
    }

    return loop_cnt;
}

uint8_t AP_RangeFinder_SK_PulsedLight::calcCheckSum(uint8_t* data, uint8_t len){
    if (len == 0){
        return 0;
    }

    uint8_t i = 0;
    uint8_t checkSum = 0;

    for (i = 0; i < len; i++){
        checkSum += data[i];
    }

    checkSum = ~checkSum;
    checkSum += 1;

    return checkSum;
}
