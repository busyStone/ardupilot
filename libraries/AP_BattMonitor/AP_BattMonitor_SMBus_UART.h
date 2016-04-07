/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_BATTMONITOR_SMBUS_UART_H
#define AP_BATTMONITOR_SMBUS_UART_H

#include "AP_BattMonitor_SMBus.h"

/* Predefined telemetry responses. */
#define TELEMETRY_RESP_OK         "_OK_"
#define TELEMETRY_RESP_FAIL       "FAIL"

/* Telemetry start-of-frame signature. */
#define TELEMETRY_MSG_SOF         0xBD
/* Empty message ID. */
#define TELEMETRY_MSG_NOMSG       0x00
/* Telemetry buffer size in bytes.  */
#define TELEMETRY_BUFFER_SIZE     0x80
/* Telemetry message header size in bytes.  */
#define TELEMETRY_MSG_HDR_SIZE    0x04
/* Telemetry message checksum size in bytes.  */
#define TELEMETRY_MSG_CRC_SIZE    0x04
/* Telemetry message header + crc size in bytes.  */
#define TELEMETRY_MSG_SVC_SIZE    ( TELEMETRY_MSG_HDR_SIZE + TELEMETRY_MSG_CRC_SIZE )

typedef struct tagTelemetryMessage {
    uint8_t sof;
    uint8_t msg_id;
    uint8_t size;
    uint8_t res;
    char data[TELEMETRY_BUFFER_SIZE];
    uint32_t crc;
} TelemetryMessage;

class AP_BattMonitor_SMBus_UART : public AP_BattMonitor_SMBus
{
public:

    // Constructor
    AP_BattMonitor_SMBus_UART(AP_BattMonitor &mon, uint8_t instance, AP_BattMonitor::BattMonitor_State &mon_state);

    /// init
    void init();
    // Read the battery voltage and current.  Should be called at 10hz
    void read();
    uint8_t capacity_remaining_pct() const;

private:

    uint32_t crc32(uint32_t pBuf[], size_t length);
    uint32_t getCRC32Checksum(TelemetryMessage *pMsg);
    void readSerialDataResync(uint8_t len);
    void processMessage();
    void sendUpdateRequestMsg(uint8_t msg_id);
    void update();
    void updateRequest();
    size_t read_port(uint8_t *buf, uint32_t len);

    AP_HAL::UARTDriver *_port;
    TelemetryMessage _msg;
    uint8_t *_msgPos;
    size_t _bytesRequired;
    uint16_t _voltage;
    int16_t  _current_amps;
    uint16_t _current_remaining_mah;
    uint16_t _percentage;
};

#endif // AP_BATTMONITOR_SMBUS_UART_H
