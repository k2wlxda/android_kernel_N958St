#ifndef T4K35_OTP_H
#define T4K35_OTP_H

#include "msm_sensor.h"

extern uint16_t  t4k35_af_macro_value; 
extern uint16_t  t4k35_af_inifity_value;
extern uint16_t  t4k35_af_otp_status;

void t4k35_otp_init_setting(struct msm_sensor_ctrl_t *s_ctrl);
#endif