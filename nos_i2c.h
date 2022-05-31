/*
 ****************************************************************************************
 * nos_i2c.h
 *
 * Author    : Infinicomm Solutions
 * Ver       : 1.0
 * Date      : 19-July-2019
 *
 * Copyright Infinicomm Solutions Ltd, 2019-21
 *
 ****************************************************************************************
 */

#ifndef NOS_I2C_H
#define NOS_I2C_H

#include "nos_types.h"

#define MAX_IO_EXPANDERS_PER_BUS                 255

typedef struct _i2c_bus_db_ {
    nos_uchar     bus_no;
    nos_int32     file_fd;
    nos_uchar     device_register[MAX_IO_EXPANDERS_PER_BUS];
    void          *pd_handle[MAX_IO_EXPANDERS_PER_BUS];
} I2C_BUS_DB;


nos_int32  nos_i2c_bus_open(nos_uchar bus_number);
nos_int32  nos_i2c_dev_close(nos_uchar bus_no);
nos_int32  nos_i2c_device_open(nos_uchar bus_number, nos_uchar device_addr);
nos_int32  nos_i2c_device_close(nos_uchar bus_number, nos_uchar device_addr);
nos_int32  nos_i2c_read(nos_uchar bus, nos_uchar i2c_addr, nos_uchar reg_addr,
                        nos_uchar *buf, nos_uint16 size);
nos_int32  nos_i2c_read_byte(nos_uchar bus, nos_uchar i2c_addr, nos_uchar *buf, nos_uint16 size);
nos_int32  nos_i2c_read_word(nos_uchar bus, nos_uchar i2c_addr, nos_uchar reg_addr, nos_uint16 *data);
nos_int32  nos_i2c_write(nos_uchar bus, nos_uchar i2c_addr, 
                        nos_uchar addr, nos_uchar *data, nos_uchar length);
nos_int32  nos_i2c_write_word(nos_uchar bus, nos_uchar i2c_addr, 
                        nos_uchar reg_addr, nos_uint16 data);
nos_int32 icos_io_init(nos_uchar bus_num, nos_uchar i2c_addr);
nos_int32 icos_io_set_direction(nos_uchar bus_number, nos_uchar i2c_address,
                               nos_uchar direction, nos_uchar init_value);
nos_int32 icos_io_set_port(nos_uchar bus_number, nos_uchar i2c_address,         
                           nos_uchar port_num, nos_uchar set_flag);

#endif

