
#ifndef __AP_INERTIAL_SENSOR_USER_INTERACT_MAVLINK_H__
#define __AP_INERTIAL_SENSOR_USER_INTERACT_MAVLINK_H__

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AP_InertialSensor_UserInteract.h"

class GCS_MAVLINK;

/**
 * AP_InertialSensor_UserInteract, implemented in terms of a MAVLink connection
 */
class AP_InertialSensor_UserInteract_MAVLink : public AP_InertialSensor_UserInteract {
public:
    AP_InertialSensor_UserInteract_MAVLink(GCS_MAVLINK* gcs_read, GCS_MAVLINK* gcs_send) :
    _gcs_read(gcs_read),
    _gcs_send(gcs_send) {}

    bool blocking_read();
    void _printf_P(const prog_char *, ...);

private:
    GCS_MAVLINK* _gcs_read;
    GCS_MAVLINK* _gcs_send; // array, size = MAVLINK_COMM_NUM_BUFFERS

    void _send_text(GCS_MAVLINK* gcs, char* msg, bool wait_uart);
};

#endif // __AP_INERTIAL_SENSOR_USER_INTERACT_MAVLINK_H__

