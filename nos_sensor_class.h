/*
 ****************************************************************************************
 * nos_sensor_class.h
 *
 * Author    : Infinicomm Technologies
 * Ver       : 1.0
 * Date      : 03-Jan-2019
 *
 * Copyright Infinicomm Technologies Ltd, 2019-20
 *
 ****************************************************************************************
 */

#ifndef _NOS_SENSOR_CLASS_H_
#define _NOS_SENSOR_CLASS_H_

/*
 * All sensor classes are defined here. All the resources in the system will fall under
 * once of these classes.
 *
 * NOS_SENSOR_DATA_CLASS  :   All sensors in the system will come under this class.
 *                                 eg. temp sensors in the system
 *
 * NOS_SYS_RESOURCES_DATA_CLASS : All system resources in the system.
 *                                     eg. Total memory available in the system.
 *
 * NOS_LOGICAL_RESOURCES_SENSOR_CLASS  : All logical interfaces in the system.
 *
 *
 */

#define NOS_OBJ_CLASS_SENSORS                                              0x00000001
#define NOS_OBJ_CLASS_SYS_RESOURCES                                        0x00000002
#define NOS_OBJ_CLASS_SNMP_RESOURCES                                       0x00000003

/*
 **************************************************************************************
 * All SENSOR SUB_CLASS are defined here.
 **************************************************************************************
 */
#define NOS_OBJ_SENSOR_SUB_CLASS_TEMP_SENSORS                              0x00000001
#define NOS_OBJ_SENSOR_SUB_CLASS_PRESSURE_SENSORS                          0x00000002
#define NOS_OBJ_SENSOR_SUB_CLASS_HUMIDITY_SENSORS                          0x00000003
#define NOS_OBJ_SENSOR_SUB_CLASS_VOLTAGE_SENSORS                           0x00000004
#define NOS_OBJ_SENSOR_SUB_CLASS_CURRENT_SENSORS                           0x00000005
#define NOS_OBJ_SENSOR_SUB_CLASS_FREQUENCY_SENSORS                         0x00000006
#define NOS_OBJ_SENSOR_SUB_CLASS_POWER_SOURCE                              0x00000007
#define NOS_OBJ_SENSOR_SUB_CLASS_POWER_SENSORS                             0x00000008
#define NOS_OBJ_SENSOR_SUB_CLASS_GENSET_SENSORS                            0x00000009
#define NOS_OBJ_SENSOR_SUB_CLASS_BATTERY_SENSORS                           0x00000010
#define NOS_OBJ_SENSOR_SUB_CLASS_UPS_SENSORS                               0x00000011
#define NOS_OBJ_SENSOR_SUB_CLASS_AIRCON_SENSORS                            0x00000012
#define NOS_OBJ_SENSOR_SUB_CLASS_FAN_SENSORS                               0x00000013
#define NOS_OBJ_SENSOR_SUB_CLASS_SMOKE_SENSORS                             0x00000014
#define NOS_OBJ_SENSOR_SUB_CLASS_DOOR_SENSORS                              0x00000015
#define NOS_OBJ_SENSOR_SUB_CLASS_MOTION_SENSORS                            0x00000016
#define NOS_OBJ_SENSOR_SUB_CLASS_DIO_SENSORS                               0x00000017
#define NOS_OBJ_SENSOR_SUB_CLASS_MODBUS_SENSORS                            0x00000018
#define NOS_OBJ_SENSOR_SUB_CLASS_ADC_SENSORS                               0x00000019
#define NOS_OBJ_SENSOR_SUB_CLASS_ATS_SENSORS                            0x00000009

#define NOS_OBJ_SENSOR_SUB_CLASS_GENERIC_SENSORS                           0x000000FF

/*
 **************************************************************************************
 *  All SYS SUB_CLASS are defined here.
 **************************************************************************************
 */
#define NOS_OBJ_SYS_RES_SUB_CLASS_CPU_INFO                                 0x00000001


/*
 **************************************************************************************
 *  All SNMP SUB_CLASS are defined here.
 **************************************************************************************
 */
#define NOS_OBJ_SNMP_SUB_CLASS_MIB_SCALARS                                 0x00000001

#endif

