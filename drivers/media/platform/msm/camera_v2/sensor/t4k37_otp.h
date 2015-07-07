#ifndef T4K37_OTP_H
#define T4K37_OTP_H

#include "msm_sensor.h"

extern  uint16_t  af_macro_value; 
extern  uint16_t  af_inifity_value;
extern  uint16_t  af_otp_status;

void t4k37_otp_init_setting(struct msm_sensor_ctrl_t *s_ctrl);
int32_t  t4k37_read_AF_OTP(struct msm_sensor_ctrl_t *s_ctrl,uint16_t *af_macro,uint16_t*af_inifity);
#endif