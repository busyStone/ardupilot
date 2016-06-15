
#include "Copter.h"


static AP_InertialSensor_UserInteract_MAVLink* m_mav_interact;

void Copter::check_switchs_user(void){
    // check compass cal
    if (!ins.calibrating()){
        if (_check_switchs_cal(&_check_compass_cal, g.rc_5.radio_in)){
            if (!compass.start_calibration_all(false, true, 0.2, false)) {
                AP_Notify::flags.compass_cal_failed = 1;
            }
        }
    }

    // check accel cal
    if (!compass.is_calibrating()){
        if (_check_switchs_cal(&_check_accel_cal, g.rc_7.radio_in)){
            if (m_mav_interact == NULL){
                m_mav_interact = new AP_InertialSensor_UserInteract_MAVLink(NULL, gcs);
            }

            ins.calibrate_accel_auto(m_mav_interact);
        }
    }
}

bool Copter::_check_switchs_cal(check_switchs_cal_s* param, int16_t radio_in){

    bool is_failsafe = failsafe.radio || failsafe.radio_counter != 0;

    // to fail fast
    if (hal.util->get_soft_armed() || is_failsafe) {
        return false;
    }

    uint32_t tnow_ms = millis();

    uint8_t pos = read_3pos_switch(radio_in);

    if (param->last_switch_position == -1) { // wait to start check

        if (pos == AUX_SWITCH_LOW){
            param->last_switch_position = pos;
            param->last_edge_time_ms = tnow_ms;
        }

        return false;
    }


    if (param->last_switch_position == AUX_SWITCH_LOW) { // wait for HIGH
        if (pos == AUX_SWITCH_HIGH) {
            if (tnow_ms - param->last_edge_time_ms < 200) {
                param->last_switch_position = pos;
                param->last_edge_time_ms = tnow_ms;
            } else {
                param->last_switch_position = -1; // reset to wait start
                param->switch_cnt = 0;
            }
        }

        return false;
    }

    if (param->last_switch_position == AUX_SWITCH_HIGH) { // wait for LOW
        if (pos == AUX_SWITCH_LOW) {
            if (tnow_ms - param->last_edge_time_ms < 200) {
                param->last_switch_position = pos;
                param->last_edge_time_ms = tnow_ms;

                param->switch_cnt++;
            } else {
                param->last_switch_position = -1; // reset to wait start
                param->switch_cnt = 0;
            }

            if (param->switch_cnt >= 3) {
                param->last_switch_position = -1; // reset to wait start
                param->switch_cnt = 0;

                return true;
            }
        }

         return false;
    }
}

void Copter::do_ch7_user_function(uint8_t ch_flag){

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V3
    if (ch_flag == AUX_SWITCH_HIGH) {
        // engage RTL (if not possible we remain in current flight mode)
        set_mode(LAND);
    }else if (ch_flag == AUX_SWITCH_MIDDLE){
        set_mode(RTL);
    }else{
        // return to flight mode switch's flight mode if we are currently in RTL
        if (control_mode == RTL || control_mode == LAND) {
            reset_control_switch();
        }
    }
#endif

}
