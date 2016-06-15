// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdarg.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AP_InertialSensor_UserInteract_MAVLink.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// set by _snoop on COMMAND_ACK
static bool _got_ack;

/*
  watch for COMMAND_ACK messages
 */
static void _snoop(const mavlink_message_t* msg)
{
    if (msg->msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
        _got_ack = true;
    }
}

bool AP_InertialSensor_UserInteract_MAVLink::blocking_read(void) 
{
    if (_gcs_read == NULL){
        return false;
    }

    uint32_t start_ms = hal.scheduler->millis();
    // setup snooping of packets so we can see the COMMAND_ACK
    _gcs_read->set_snoop(_snoop);
    _got_ack = false;
    while (hal.scheduler->millis() - start_ms < 30000U) {
        hal.scheduler->delay(10);
        if (_got_ack) {
            _gcs_read->set_snoop(NULL);
            return true;    
        }
    }
    hal.console->println_P(PSTR("Timed out waiting for user response"));
    _gcs_read->set_snoop(NULL);
    return false;
}

void AP_InertialSensor_UserInteract_MAVLink::_printf_P(const prog_char* fmt, ...) 
{
    char msg[50];
    va_list ap;
    va_start(ap, fmt);
    hal.util->vsnprintf_P(msg, sizeof(msg), (const prog_char_t *)fmt, ap);
    va_end(ap);
    if (msg[strlen(msg)-1] == '\n') {
        // STATUSTEXT messages should not add linefeed
        msg[strlen(msg)-1] = 0;
    }

    if (_gcs_read != NULL){
        _send_text(_gcs_read, msg, true);

        return;
    }

    if (_gcs_send == NULL){
        return;
    }

    for (uint8_t i=0; i<MAVLINK_COMM_NUM_BUFFERS; i++) {
        if (_gcs_send[i].initialised) {
            _send_text(&_gcs_send[i], msg, false);
        }
    }
}

void AP_InertialSensor_UserInteract_MAVLink::_send_text(GCS_MAVLINK* gcs, char* msg, bool wait_uart){

    if (wait_uart){
        AP_HAL::UARTDriver *uart = gcs->get_uart();

        /*
          to ensure these messages get to the user we need to wait for the
          port send buffer to have enough room
         */
        while (uart->txspace() < MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_STATUSTEXT_LEN) {
            hal.scheduler->delay(1);
        }
    }

    gcs->send_text_P(SEVERITY_HIGH, msg);
}

