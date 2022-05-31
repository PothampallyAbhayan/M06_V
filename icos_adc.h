/*
 ****************************************************************************************
 * icos_adc.h
 *
 * Author    : Infinicomm Technologies Lts
 * Ver       : 1.0
 * Date      : 06-Jan-2019
 *
 * Copyright Infinicomm Technologies Pvt Ltd, 2019-2020
 *
 ****************************************************************************************
 */

#ifndef _ICOS_ADC_H_
#define _ICOS_ADC_H_


#include "nos_types.h"

nos_int32 icos_adc_init (nos_uchar  bus_number, nos_uchar  i2c_address);

nos_int32  icos_adc_read_channel(nos_uchar  bus_number,
                                 nos_uchar  i2c_address,
                                 nos_uchar  channel_id,
                                 nos_int16  *data);

#endif    
    

