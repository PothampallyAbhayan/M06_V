/*
 ****************************************************************************************
 * icos_pwm.h
 *
 * Author    : Infinicomm Technologies Lts
 * Ver       : 1.0
 * Date      : 06-Jan-2019
 *
 * Copyright Infinicomm Technologies Pvt Ltd, 2019-2020
 *
 ****************************************************************************************
 */

#ifndef _ICOS_PWM_H_
#define _ICOS_PWM_H_


#include "nos_types.h"

#define PWM_CHANNEL_0                        0x00
#define PWM_CHANNEL_1                        0x01
#define PWM_CHANNEL_2                        0x02
#define PWM_CHANNEL_3                        0x03
#define PWM_CHANNEL_4                        0x04
#define PWM_CHANNEL_5                        0x05
#define PWM_CHANNEL_6                        0x06
#define PWM_CHANNEL_7                        0x07

/*
 * All API templates are declared here.
 */
nos_int32 icos_pwm_init (nos_uchar  pwm_channel);
nos_uint16  icos_set_pwm_period(nos_uchar  pwm_channel, nos_uint32  period);
nos_uint16  icos_set_pwm_duty_cycle(nos_uchar  pwm_channel, nos_uint32  duty_cycle);
nos_uint16  icos_pwm_channel_enable(nos_uchar  pwm_channel, nos_uchar en_dis_flag);


#endif    
    

