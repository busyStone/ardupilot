/*
 *  Example of AP_OpticalFlow library.
 *  Code by Randy Mackay. DIYDrones.com
 */

#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <AP_HAL_SITL/AP_HAL_SITL.h>
#include <AP_HAL_PX4/AP_HAL_PX4.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Notify/AP_Notify.h>
#include <DataFlash/DataFlash.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_ADC/AP_ADC.h>
#include <AP_ADC_AnalogSource/AP_ADC_AnalogSource.h>
#include <Filter/Filter.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_BattMonitor/AP_BattMonitor.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

static OpticalFlow optflow;

void setup()
{
    hal.console->println("OpticalFlow library test ver 1.6");

    hal.scheduler->delay(1000);

    // flowSensor initialization
    optflow.init();

    hal.scheduler->delay(1000);
}

void loop()
{
    hal.console->println("in loop");

    optflow.update();

    if (!optflow.healthy()) {
        hal.console->println("Failed to initialise PX4Flow ");
    }else{
        hal.console->printf("device id %d\n", optflow.test_state.device_id);
        hal.console->printf("data surface_quality = %d\n", optflow.test_state.surface_quality);
        hal.console->printf("data flowRate.x = %f\n", optflow.test_state.flowRate.x);
        hal.console->printf("data flowRate.y = %f\n", optflow.test_state.flowRate.y);
        hal.console->printf("data bodyRate.x = %f\n", optflow.test_state.bodyRate.x);
        hal.console->printf("data bodyRate.y = %f\n", optflow.test_state.bodyRate.y);
    }


    hal.scheduler->delay(200);
}

AP_HAL_MAIN();
