/*
  Battery SMBus UART driver
*/
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

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 && !defined(CONFIG_ARCH_BOARD_PX4FMU_V3)

#include "AP_BattMonitor_SMBus_UART.h"
#include <stdio.h>
#include <math.h>

extern const AP_HAL::HAL& hal;

#define IS_MSG_VALID() \
  ((_msg.sof == TELEMETRY_MSG_SOF) &&\
  (_msg.size >= TELEMETRY_MSG_SVC_SIZE) &&\
  (_msg.size <= TELEMETRY_BUFFER_SIZE))

/* Nibble lookup table for 0x04C11DB7 polynomial. */
static const uint32_t crc_tab[16] = {
    0x00000000,0x04C11DB7,0x09823B6E,0x0D4326D9,
    0x130476DC,0x17C56B6B,0x1A864DB2,0x1E475005,
    0x2608EDB8,0x22C9F00F,0x2F8AD6D6,0x2B4BCB61,
    0x350C9B64,0x31CD86D3,0x3C8EA00A,0x384FBDBD
};

// Constructor
AP_BattMonitor_SMBus_UART::AP_BattMonitor_SMBus_UART(AP_BattMonitor &mon, uint8_t instance, 
    AP_BattMonitor::BattMonitor_State &mon_state,
    AP_HAL::UARTDriver *port) :
        AP_BattMonitor_SMBus(mon, instance, mon_state),
        _port(port),
        _bytesRequired(TELEMETRY_MSG_HDR_SIZE),
        _temperature(0),
        _voltage(0),
        _current_amps(0),
        _current_remaining_mah(0),
        _percentage(0)
{
}

void AP_BattMonitor_SMBus_UART::init()
{
    _port->begin(
        AP_SERIALMANAGER_SK_BATTERY_BAUD,
        AP_SERIALMANAGER_SK_BATTERY_BUFSIZE_RX,
        AP_SERIALMANAGER_SK_BATTERY_BUFSIZE_TX);

    _msgPos = (uint8_t *)&_msg;
}

// read - read latest voltage and current
void AP_BattMonitor_SMBus_UART::read()
{
    uint32_t tnow = hal.scheduler->micros();

    updateRequest();
    update();

    _state.temperature = (float)_temperature / 100.0f;
    _state.voltage = (float)_voltage / 1000.0f;
    _state.current_amps = fabsf(_current_amps) / 1000.0f;
    _state.current_total_mah =
        (
            (float)(_current_remaining_mah * 100.0f) / (float)_percentage
          - (float)_current_remaining_mah
        ) / 1000.0f;
    _state.last_time_micros = tnow;
    _state.healthy = true;

    // timeout after 5 seconds
    if ((tnow - _state.last_time_micros) > AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS) {
        _state.healthy = false;
    }
}

/// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100)
uint8_t AP_BattMonitor_SMBus_UART::capacity_remaining_pct() const
{
    return _percentage;
}

/**
 * @brief crc32 - calculates CRC32 checksum of the data buffer.
 * @param pBuf - address of the data buffer.
 * @param length - length of the buffer.
 * @return CRC32 checksum.
 */
uint32_t AP_BattMonitor_SMBus_UART::crc32(uint32_t pBuf[], size_t length)
{
    uint32_t i;
    /* Initial XOR value. */
    uint32_t crc = 0xFFFFFFFF;

    for (i = 0; i < length; i++) {
        /* Apply all 32-bits: */
        crc ^= pBuf[i];

        /* Process 32-bits, 4 at a time, or 8 rounds.
         * - Assumes 32-bit reg, masking index to 4-bits;
         * - 0x04C11DB7 Polynomial used in STM32.
         */
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
    }
    return crc;
}

uint32_t AP_BattMonitor_SMBus_UART::getCRC32Checksum(TelemetryMessage *pMsg) {
    size_t crc_length = (pMsg->size - TELEMETRY_MSG_CRC_SIZE) / sizeof(uint32_t);
    if ((pMsg->size - TELEMETRY_MSG_CRC_SIZE) % sizeof(uint32_t)) {
        crc_length++;
    }
    return crc32((uint32_t *)pMsg, crc_length);
}

void AP_BattMonitor_SMBus_UART::readSerialDataResync(uint8_t len) {
    uint8_t i;

    while (len) {
        for (i = 1; i < len; i++) {
            if (((uint8_t *)&_msg)[i] == TELEMETRY_MSG_SOF) {
                break;
            }
        }

        if (len - i > 0) {
            memmove(&_msg, &((uint8_t *)&_msg)[i], len - i);
        }
        len -= i;
        _msgPos = (uint8_t *)&_msg + len;

        if (len < TELEMETRY_MSG_HDR_SIZE) {
            /* ...wait for header to get completed */
            _bytesRequired = TELEMETRY_MSG_HDR_SIZE - len;
            break;
        } else {
            if (IS_MSG_VALID()) {
                if (_msg.size <= len) {
                    /* This may throw away some data at the tail of buffer...*/
                    _bytesRequired = 0;
                } else {
                    _bytesRequired = _msg.size - len;
                }
                break;
            }
        }
    }
}

void AP_BattMonitor_SMBus_UART::processMessage() {
    switch (_msg.msg_id) {
    case 't':
        _temperature = *(uint16_t *)_msg.data;
    case 'v':
        _voltage = *(uint16_t *)_msg.data;
        //printf("num_gcs:%d\n", num_gcs);
        break;
    case 'c':
        _current_amps = *(int16_t *)_msg.data;
        //printf("c:%d\n", _current_amps);
        break;
    case 'p':
        _percentage = *(uint16_t *)_msg.data;
        //printf("p:%d\n", _percentage);
        break;
    case 'r':
        _current_remaining_mah = *(uint16_t*)_msg.data;
        //printf("r:%d\n", _current_remaining_mah);
        break;
    case 's':
        break;
    default:
        break;
    }
    return;
}

void AP_BattMonitor_SMBus_UART::sendUpdateRequestMsg(uint8_t msg_id) {
    TelemetryMessage msg;

    memset(&msg, 0, sizeof(TelemetryMessage));

    msg.sof  = TELEMETRY_MSG_SOF;
    msg.size = TELEMETRY_MSG_SVC_SIZE;
    msg.res  = 0;

    msg.msg_id = msg_id;
    msg.crc = getCRC32Checksum(&msg);

    _port->write((const uint8_t *)&msg, msg.size - TELEMETRY_MSG_CRC_SIZE);
    _port->write((const uint8_t *)&msg.crc, TELEMETRY_MSG_CRC_SIZE);

    return;
}

void AP_BattMonitor_SMBus_UART::updateRequest() {
    sendUpdateRequestMsg('t');
    sendUpdateRequestMsg('v');
    sendUpdateRequestMsg('c');
    sendUpdateRequestMsg('p');
    sendUpdateRequestMsg('r');

}

void AP_BattMonitor_SMBus_UART::update() {

    /* The following function must be called from within a system lock zone. */
    size_t bytesAvailable = _port->available();

    while (bytesAvailable) {
        if (bytesAvailable >= _bytesRequired) {
            if (_bytesRequired > 0) {
                read_port(_msgPos, _bytesRequired);
                _msgPos += _bytesRequired;
                bytesAvailable -= _bytesRequired;
                _bytesRequired = 0;
            }
        } else {
            read_port(_msgPos, bytesAvailable);
            _msgPos += bytesAvailable;
            _bytesRequired -= bytesAvailable;
            break;
        }

        size_t curReadLen = _msgPos - (uint8_t *)&_msg;
        if (!IS_MSG_VALID()) {
            readSerialDataResync(curReadLen);
        } else if (curReadLen == TELEMETRY_MSG_HDR_SIZE) {
            _bytesRequired = _msg.size - TELEMETRY_MSG_HDR_SIZE;
        } else if (_bytesRequired == 0) {
            /* Whole packet is read, check and process it... */
            /* Move CRC */
            memmove(&_msg.crc, (uint8_t *)&_msg + _msg.size - TELEMETRY_MSG_CRC_SIZE,
                    TELEMETRY_MSG_CRC_SIZE);
            /* Zero-out unused data for crc32 calculation. */
            memset(&_msg.data[_msg.size - TELEMETRY_MSG_SVC_SIZE], 0,
                   TELEMETRY_BUFFER_SIZE - _msg.size + TELEMETRY_MSG_SVC_SIZE);

            if (_msg.crc == getCRC32Checksum(&_msg)) {
                processMessage();
            } else {
                /* do nothing */
            }

            /* Read another packet...*/
            _bytesRequired = TELEMETRY_MSG_HDR_SIZE;
            _msgPos = (uint8_t *)&_msg;
        }
    }
}

size_t AP_BattMonitor_SMBus_UART::read_port(uint8_t *buf, uint32_t len) {
    uint32_t i;

    if (_port->available() < (int16_t)len) {
        return 0;
    }

    for (i = 0; i < len; i++) {
        buf[i] = _port->read();
    }

    return i;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4
