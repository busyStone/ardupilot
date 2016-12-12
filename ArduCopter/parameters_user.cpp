
#include "Copter.h"

void Copter::load_parameters_user(void){
    // frame type
    // flight mode
    // batt monitor
    // rc rssi
    // rc2 reverse
    // thr_mid
    // failsafe
    // compass cal fit
    // compass orient
    // serial2 baund
    // rc3 rc4 dead zone
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V3
    set_parameters_user((char*)"FRAME", 1.0f); // x frame

    set_parameters_user((char*)"FLTMODE1", 16.0f); // pos hold
    set_parameters_user((char*)"FLTMODE3", 2.0f); // alt hold
    set_parameters_user((char*)"FLTMODE6", 0.0f); // stabilize

    set_parameters_user((char*)"BATT_MONITOR", 3.0f); // voltage only
    set_parameters_user((char*)"BATT_VOLT_PIN", 2.0f); // Pixhawk
    set_parameters_user((char*)"BATT_VOLT_MULT", 4.187161f);

    set_parameters_user((char*)"RSSI_PIN", 11.0f);
    set_parameters_user((char*)"RSSI_RANGE", 3.3f);

    set_parameters_user((char*)"RC2_REV", -1.0f);

    set_parameters_user((char*)"THR_MID", 455.0f);

    set_parameters_user((char*)"FS_BATT_ENABLE", 2.0f); // RTL
    set_parameters_user((char*)"FS_BATT_VOLTAGE", 10.6f); // 10.6v
    set_parameters_user((char*)"FS_EKF_ACTION", 1.0f); // land
    set_parameters_user((char*)"FS_GCS_ENABLE", 1.0f); // always RTL
    set_parameters_user((char*)"FS_THR_ENABLE", 1.0f); // always RTL

    set_parameters_user((char*)"COMPASS_CAL_FIT", 16.0f);

    set_parameters_user((char*)"PHLD_BRAKE_ANGLE", 4200);
    set_parameters_user((char*)"PHLD_BRAKE_RATE", 11);

    // incompatible changes frome 1.0.2.7
    set_parameters_user((char*)"COMPASS_ORIENT", 16.0f);
    set_parameters_user((char*)"SERIAL2_BAUD", 115);

    // set_parameters_user((char*)"RC1_DZ", 80);
    // set_parameters_user((char*)"RC2_DZ", 80);

#endif
}

void Copter::set_parameters_user(char* key, float value){
    // set parameter
    AP_Param *vp;
    enum ap_var_type var_type;

    // find existing param so we can get the old value
    vp = AP_Param::find(key, &var_type);
    if (vp == NULL) {
        return;
    }
    float old_value = vp->cast_to_float(var_type);

    // set the value
    vp->set_float(value, var_type);

    /*
      we force the save if the value is not equal to the old
      value. This copes with the use of override values in
      constructors, such as PID elements. Otherwise a set to the
      default value which differs from the constructor value doesn't
      save the change
     */
    bool force_save = !is_equal(value, old_value);

    // save the change
    vp->save(force_save);
}