/*
 ****************************************************************************************
 * nos_sensor_attributes.h
 *
 * Author    : Infinicomm Technologies
 * Ver       : 1.0
 * Date      : 03-Jan-2019
 *
 * Copyright Infinicomm Technologies Ltd, 2019-20
 *
 ****************************************************************************************
 */

#ifndef _NOS_SENSOR_ATTRIBUTES_H_
#define _NOS_SENSOR_ATTRIBUTES_H_

/*
 * All sensor attributes of sensorMIB are defined here. The following are its values.
 *
 *  nOSsensorIndex
 *  nOSsensorName
 *  nOSsensorType
 *  nOSsensorScale
 *  nOSsensorPrecision
 *  nOSsensorValue
 *  nOSsensorStatus
 *  nOSsensorValueTimeStamp
 *  nOSsensorvalueUpdateRate
 *  nOSactuatorIndex
 *  nOSactuatorName
 *  nOSactuatorValue
 *
 */

#define NOS_SENSOR_INDEX_ATTRIB_ID                    0x00000001
#define NOS_SENSOR_NAME_ATTRIB_ID                     ( NOS_SENSOR_INDEX_ATTRIB_ID + 1 )
#define NOS_SENSOR_TYPE_ATTRIB_ID                     ( NOS_SENSOR_NAME_ATTRIB_ID + 1 )
#define NOS_SENSOR_SCALE_ATTRIB_ID                    ( NOS_SENSOR_TYPE_ATTRIB_ID + 1 )
#define NOS_SENSOR_PRECISION_ATTRIB_ID                ( NOS_SENSOR_SCALE_ATTRIB_ID + 1 )
#define NOS_SENSOR_VALUE_ATTRIB_ID                    ( NOS_SENSOR_PRECISION_ATTRIB_ID + 1 )
#define NOS_SENSOR_STATUS_ATTRIB_ID                   ( NOS_SENSOR_VALUE_ATTRIB_ID + 1 )
#define NOS_SENSOR_VALUE_TIMESTAMP_ATTRIB_ID          ( NOS_SENSOR_STATUS_ATTRIB_ID + 1 )
#define NOS_SENSOR_UPDATE_RATE_ATTRIB_ID              ( NOS_SENSOR_VALUE_TIMESTAMP_ATTRIB_ID + 1 )

/*
 * actuatorTable
 */

#define NOS_ACTUATOR_INDEX_ATTRIB_ID                  ( NOS_SENSOR_UPDATE_RATE_ATTRIB_ID + 1 )
#define NOS_ACTUATOR_NAME_ATTRIB_ID                   ( NOS_ACTUATOR_INDEX_ATTRIB_ID + 1 )
#define NOS_ACTUATOR_VALUE_ATTRIB_ID                  ( NOS_ACTUATOR_NAME_ATTRIB_ID + 1 )

/*
 * modbusTable
 */

#define NOS_MODBUS_REGISTER_INDEX_ATTRIB_ID           ( NOS_ACTUATOR_VALUE_ATTRIB_ID + 1 )
#define NOS_MODBUS_BUS_NUM_ATTRIB_ID                  ( NOS_MODBUS_REGISTER_INDEX_ATTRIB_ID + 1 )
#define NOS_MODBUS_SLAVE_ADDR_ATTRIB_ID               ( NOS_MODBUS_BUS_NUM_ATTRIB_ID + 1 )
#define NOS_MODBUS_REGISTER_ADDR_ATTRIB_ID            ( NOS_MODBUS_SLAVE_ADDR_ATTRIB_ID + 1 )
#define NOS_MODBUS_REGISTER_TYPE_ATTRIB_ID            ( NOS_MODBUS_REGISTER_ADDR_ATTRIB_ID + 1 )
#define NOS_MODBUS_REGISTER_VALUE_ATTRIB_ID           ( NOS_MODBUS_REGISTER_TYPE_ATTRIB_ID + 1 )

/*
 * AdcTable
 */

#define NOS_ADC_CHANNEL_INDEX_ATTRIB_ID               ( NOS_MODBUS_REGISTER_VALUE_ATTRIB_ID + 1 )
#define NOS_ADC_CHANNEL_ID_ATTRIB_ID                  ( NOS_ADC_CHANNEL_INDEX_ATTRIB_ID + 1 )
#define NOS_ADC_CHANNEL_NAME_ATTRIB_ID                ( NOS_ADC_CHANNEL_ID_ATTRIB_ID + 1 )
#define NOS_ADC_CHANNEL_VALUE_ATTRIB_ID               ( NOS_ADC_CHANNEL_NAME_ATTRIB_ID + 1 )

/*
 * All sensor attributes of sensorMIB are defined here. The following are its values.
 *
 *  pwrSupplySrcIndex
 *  pwrSupplySrcName
 *  pwrSupplySrcValue
 *  pwrSupplySrcStatus
 *  supplyVoltageR
 *  supplyVoltageY
 *  supplyVoltageB
 *  supplyCurrentR
 *  supplyCurrentY
 *  supplyCurrentB
 *  supplykwR
 *  supplykwY
 *  supplykwB
 *  supplykwHour
 *  supplyPowerFactorR
 *  supplyPowerFactorY
 *  supplyPowerFactorB
 *  powerSupplyFrequency
 *  powerSupplyRunHrs
 *  pwrSrcIndex
 *  pwrSrcName
 *  pwrSrcName
 *  pwrSrcStatus
 *  voltageR
 *  voltageY
 *  voltageB
 *  currentR
 *  currentY
 *  currentB
 *  kwR
 *  kwY
 *  kwB
 *  kwHour
 *  powerFactorR
 *  powerFactorY
 *  powerFactorB
 *  frequency
 *  runHrs
 *  gensetIndex
 *  gensetName
 *  gensetRpm
 *  gensetTemp
 *  gensetFuelLevel
 *  gensetCoolantTemp
 *  gensetCoolantLevel
 *  gensetCoolantPressure
 *  gensetOilTemp
 *  gensetOilLevel
 *  gensetOilPressure
 *  gensetOilViscosity
 *  gensetBatteryVoltage
 *  gensetLLOP
 *  gensetShelterTemp
 *  gensetRunMode
 *  batteryIndex
 *  batteryVoltage
 *  batteryCurrent
 *  batteryTemp
 *  upsIndex
 *  upsName
 *  upsStatus
 *  upsBatteryStatus
 *  upsSecOnBattery
 *  upsResidualMins
 *  upsResidualCharge
 *  upsBatteryVoltage
 *  upsBatteryCurrent
 *  upsBatteryTemp
 *  airconIndex
 *  airconTemp
 *  airconStatus
 *  airconVoltage
 *  airconCurrent
 *  airconPower
 *  fanIndex
 *  fanStatus
 *  fanVoltage
 *  fanCurrent
 *  fanPower
 *
 */

/*
 * powerSupplyTable
 */
#define NOS_PWR_SUPPLY_SRC_INDEX_ATTRIB_ID            ( NOS_ADC_CHANNEL_VALUE_ATTRIB_ID + 1 )
#define NOS_PWR_SUPPLY_SRC_NAME_ATTRIB_ID             ( NOS_PWR_SUPPLY_SRC_INDEX_ATTRIB_ID + 1 )
#define NOS_PWR_SUPPLY_SRC_VALUE_ATTRIB_ID            ( NOS_PWR_SUPPLY_SRC_NAME_ATTRIB_ID + 1 )
#define NOS_PWR_SUPPLY_SRC_STATUS_ATTRIB_ID           ( NOS_PWR_SUPPLY_SRC_VALUE_ATTRIB_ID + 1 )
#define NOS_SUPPLY_VOLTAGE_R_ATTRIB_ID                ( NOS_PWR_SUPPLY_SRC_STATUS_ATTRIB_ID + 1 )
#define NOS_SUPPLY_VOLTAGE_Y_ATTRIB_ID                ( NOS_SUPPLY_VOLTAGE_R_ATTRIB_ID + 1 )
#define NOS_SUPPLY_VOLTAGE_B_ATTRIB_ID                ( NOS_SUPPLY_VOLTAGE_Y_ATTRIB_ID + 1 )
#define NOS_SUPPLY_CURRENT_R_ATTRIB_ID                ( NOS_SUPPLY_VOLTAGE_B_ATTRIB_ID + 1 )
#define NOS_SUPPLY_CURRENT_Y_ATTRIB_ID                ( NOS_SUPPLY_CURRENT_R_ATTRIB_ID + 1 )
#define NOS_SUPPLY_CURRENT_B_ATTRIB_ID                ( NOS_SUPPLY_CURRENT_Y_ATTRIB_ID + 1 )
#define NOS_SUPPLY_KW_R_ATTRIB_ID                     ( NOS_SUPPLY_CURRENT_B_ATTRIB_ID + 1 )
#define NOS_SUPPLY_KW_Y_ATTRIB_ID                     ( NOS_SUPPLY_KW_R_ATTRIB_ID + 1 )
#define NOS_SUPPLY_KW_B_ATTRIB_ID                     ( NOS_SUPPLY_KW_Y_ATTRIB_ID + 1 )
#define NOS_SUPPLY_KW_HOUR_ATTRIB_ID                  ( NOS_SUPPLY_KW_B_ATTRIB_ID + 1 )
#define NOS_SUPPLY_PF_R_ATTRIB_ID                     ( NOS_SUPPLY_KW_HOUR_ATTRIB_ID + 1 )
#define NOS_SUPPLY_PF_Y_ATTRIB_ID                     ( NOS_SUPPLY_PF_R_ATTRIB_ID + 1 )
#define NOS_SUPPLY_PF_B_ATTRIB_ID                     ( NOS_SUPPLY_PF_Y_ATTRIB_ID + 1 )
#define NOS_PWR_SUPPLY_FREQ_ATTRIB_ID                 ( NOS_SUPPLY_PF_B_ATTRIB_ID + 1 )
#define NOS_PWR_SUPPLY_RUN_HRS_ATTRIB_ID              ( NOS_PWR_SUPPLY_FREQ_ATTRIB_ID + 1 )

/*
 * powerMeasurementTable
 */

#define NOS_PWR_SRC_INDEX_ATTRIB_ID                   ( NOS_PWR_SUPPLY_RUN_HRS_ATTRIB_ID + 1 )
#define NOS_PWR_SRC_NAME_ATTRIB_ID                    ( NOS_PWR_SRC_INDEX_ATTRIB_ID + 1 )
#define NOS_PWR_SRC_VALUE_ATTRIB_ID                   ( NOS_PWR_SRC_NAME_ATTRIB_ID + 1 )
#define NOS_PWR_SRC_STATUS_ATTRIB_ID                  ( NOS_PWR_SRC_VALUE_ATTRIB_ID + 1 )
#define NOS_VOLTAGE_R_ATTRIB_ID                       ( NOS_PWR_SRC_STATUS_ATTRIB_ID + 1 )
#define NOS_VOLTAGE_Y_ATTRIB_ID                       ( NOS_VOLTAGE_R_ATTRIB_ID + 1 )
#define NOS_VOLTAGE_B_ATTRIB_ID                       ( NOS_VOLTAGE_Y_ATTRIB_ID + 1 )
#define NOS_CURRENT_R_ATTRIB_ID                       ( NOS_VOLTAGE_B_ATTRIB_ID + 1 )
#define NOS_CURRENT_Y_ATTRIB_ID                       ( NOS_CURRENT_R_ATTRIB_ID + 1 )
#define NOS_CURRENT_B_ATTRIB_ID                       ( NOS_CURRENT_Y_ATTRIB_ID+ 1 )
#define NOS_KW_R_ATTRIB_ID                            ( NOS_CURRENT_B_ATTRIB_ID + 1 )
#define NOS_KW_Y_ATTRIB_ID                            ( NOS_KW_R_ATTRIB_ID + 1 )
#define NOS_KW_B_ATTRIB_ID                            ( NOS_KW_Y_ATTRIB_ID + 1 )
#define NOS_KW_HOUR_ATTRIB_ID                         ( NOS_KW_B_ATTRIB_ID + 1 )
#define NOS_PF_R_ATTRIB_ID                            ( NOS_KW_HOUR_ATTRIB_ID + 1 )
#define NOS_PF_Y_ATTRIB_ID                            ( NOS_PF_R_ATTRIB_ID + 1 )
#define NOS_PF_B_ATTRIB_ID                            ( NOS_PF_Y_ATTRIB_ID + 1 )
#define NOS_FREQUENCY_ATTRIB_ID                       ( NOS_PF_B_ATTRIB_ID + 1 )
#define NOS_RUN_HRS_ATTRIB_ID                         ( NOS_FREQUENCY_ATTRIB_ID + 1 )
#define NOS_VOL_LINE_AVG_ATTRIB_ID                    ( NOS_RUN_HRS_ATTRIB_ID + 1 )
#define NOS_VOL_LL_AVG_ATTRIB_ID                      ( NOS_VOL_LINE_AVG_ATTRIB_ID + 1 )
#define NOS_VOL_RY_AVG_ATTRIB_ID                      ( NOS_VOL_LL_AVG_ATTRIB_ID + 1 )
#define NOS_VOL_YB_AVG_ATTRIB_ID                      ( NOS_VOL_RY_AVG_ATTRIB_ID + 1 )
#define NOS_VOL_BR_AVG_ATTRIB_ID                      ( NOS_VOL_YB_AVG_ATTRIB_ID + 1 )
#define NOS_CURRENT_AVG_ATTRIB_ID                     ( NOS_VOL_BR_AVG_ATTRIB_ID + 1 )
#define NOS_PF_AVG_ATTRIB_ID                          ( NOS_CURRENT_AVG_ATTRIB_ID + 1 )


/*
 * genTable
 */
#define NOS_GENSET_INDEX_ATTRIB_ID                    ( NOS_PF_AVG_ATTRIB_ID + 1 )
#define NOS_GENSET_NAME_ATTRIB_ID                     ( NOS_GENSET_INDEX_ATTRIB_ID + 1 )
#define NOS_GENSET_RPM_ATTRIB_ID                      ( NOS_GENSET_NAME_ATTRIB_ID + 1 )
#define NOS_GENSET_TEMP_ATTRIB_ID                     ( NOS_GENSET_RPM_ATTRIB_ID + 1 )
#define NOS_GENSET_FUEL_LEVEL_ATTRIB_ID               ( NOS_GENSET_TEMP_ATTRIB_ID + 1 )
#define NOS_GENSET_COOLANT_TEMP_ATTRIB_ID             ( NOS_GENSET_FUEL_LEVEL_ATTRIB_ID + 1 )
#define NOS_GENSET_COOLANT_LEVEL_ATTRIB_ID            ( NOS_GENSET_COOLANT_TEMP_ATTRIB_ID + 1 )
#define NOS_GENSET_COOLANT_PRESSURE_ATTRIB_ID         ( NOS_GENSET_COOLANT_LEVEL_ATTRIB_ID + 1 )
#define NOS_GENSET_OIL_TEMP_ATTRIB_ID                 ( NOS_GENSET_COOLANT_PRESSURE_ATTRIB_ID + 1 )
#define NOS_GENSET_OIL_LEVEL_ATTRIB_ID                ( NOS_GENSET_OIL_TEMP_ATTRIB_ID + 1 )
#define NOS_GENSET_OIL_PRESSURE_ATTRIB_ID             ( NOS_GENSET_OIL_LEVEL_ATTRIB_ID + 1 )
#define NOS_GENSET_OIL_VISCOSITY_ATTRIB_ID            ( NOS_GENSET_OIL_PRESSURE_ATTRIB_ID + 1 )
#define NOS_GENSET_BATTERY_VOLT_ATTRIB_ID             ( NOS_GENSET_OIL_VISCOSITY_ATTRIB_ID + 1 )
#define NOS_GENSET_LLOP_ATTRIB_ID                     ( NOS_GENSET_BATTERY_VOLT_ATTRIB_ID + 1 )
#define NOS_GENSET_SHELTER_TEMP_ATTRIB_ID             ( NOS_GENSET_LLOP_ATTRIB_ID + 1 )
#define NOS_GENSET_RUN_MODE_ATTRIB_ID                 ( NOS_GENSET_SHELTER_TEMP_ATTRIB_ID + 1 )
#define NOS_GENSET_VALUE_ATTRIB_ID                    ( NOS_GENSET_RUN_MODE_ATTRIB_ID + 1 )
#define NOS_GENSET_STATUS_ATTRIB_ID                   ( NOS_GENSET_VALUE_ATTRIB_ID + 1 )
#define NOS_GENSET_DG_FUEL_PERCENTAGE_ATTRIB_ID       ( NOS_GENSET_STATUS_ATTRIB_ID + 1 )
#define NOS_GENSET_FUEL_RES_LEVEL_ATTRIB_ID           ( NOS_GENSET_DG_FUEL_PERCENTAGE_ATTRIB_ID + 1 )
#define NOS_GENSET_DG_BATTERY_VOL_ATTRIB_ID           ( NOS_GENSET_FUEL_RES_LEVEL_ATTRIB_ID + 1 )
#define NOS_GENSET_CYCLES_COUNT_ATTRIB_ID             ( NOS_GENSET_DG_BATTERY_VOL_ATTRIB_ID + 1 )
#define NOS_GENSET_DG_RPM_ATTRIB_ID                   ( NOS_GENSET_CYCLES_COUNT_ATTRIB_ID + 1 )
#define NOS_GENSET_DG_CONTROL_ATTRIB_ID               ( NOS_GENSET_DG_RPM_ATTRIB_ID + 1 )

/*
 * upsTable
 */

#define NOS_UPS_INDEX_ATTRIB_ID                       ( NOS_GENSET_DG_CONTROL_ATTRIB_ID + 1 )
#define NOS_UPS_NAME_ATTRIB_ID                        ( NOS_UPS_INDEX_ATTRIB_ID + 1 )
#define NOS_UPS_STATUS_ATTRIB_ID                      ( NOS_UPS_NAME_ATTRIB_ID + 1 )
#define NOS_UPS_VOLTAGE_ATTRIB_ID                     ( NOS_UPS_STATUS_ATTRIB_ID + 1 )
#define NOS_UPS_CURRENT_ATTRIB_ID                     ( NOS_UPS_VOLTAGE_ATTRIB_ID + 1 )
#define NOS_UPS_TEMP_ATTRIB_ID                        ( NOS_UPS_CURRENT_ATTRIB_ID + 1 )
#define NOS_UPS_SEC_ON_BATTERY_ATTRIB_ID              ( NOS_UPS_TEMP_ATTRIB_ID + 1 )
#define NOS_UPS_RESIDUAL_MINS_ATTRIB_ID               ( NOS_UPS_SEC_ON_BATTERY_ATTRIB_ID + 1 )
#define NOS_UPS_RESIDUAL_CHARGE_ATTRIB_ID             ( NOS_UPS_RESIDUAL_MINS_ATTRIB_ID + 1 )

/*
 * batteryTable
 */

#define NOS_BATTERY_INDEX_ATTRIB_ID                   ( NOS_UPS_RESIDUAL_CHARGE_ATTRIB_ID + 1 )
#define NOS_BATTERY_VOLTAGE_ATTRIB_ID                 ( NOS_BATTERY_INDEX_ATTRIB_ID + 1 )
#define NOS_BATTERY_CURRENT_ATTRIB_ID                 ( NOS_BATTERY_VOLTAGE_ATTRIB_ID + 1 )
#define NOS_BATTERY_UPS_NAME_ATTRIB_ID                ( NOS_BATTERY_CURRENT_ATTRIB_ID + 1 )
#define NOS_BATTERY_STRING_NUM_ATTRIB_ID              ( NOS_BATTERY_UPS_NAME_ATTRIB_ID + 1 )
#define NOS_BATTERY_TEMP_ATTRIB_ID                    ( NOS_BATTERY_STRING_NUM_ATTRIB_ID + 1 )
#define NOS_BATTERY_NUMBER_ATTRIB_ID                  ( NOS_BATTERY_TEMP_ATTRIB_ID + 1 )


/*
 * airconTable
 */

#define NOS_AIRCON_INDEX_ATTRIB_ID                    ( NOS_BATTERY_NUMBER_ATTRIB_ID + 1 )
#define NOS_AIRCON_TEMP_ATTRIB_ID                     ( NOS_AIRCON_INDEX_ATTRIB_ID + 1 )
#define NOS_AIRCON_STATUS_ATTRIB_ID                   ( NOS_AIRCON_TEMP_ATTRIB_ID + 1 )
#define NOS_AIRCON_VOLTAGE_ATTRIB_ID                  ( NOS_AIRCON_STATUS_ATTRIB_ID + 1 )
#define NOS_AIRCON_CURRENT_ATTRIB_ID                  ( NOS_AIRCON_VOLTAGE_ATTRIB_ID + 1 )
#define NOS_AIRCON_POWER_ATTRIB_ID                    ( NOS_AIRCON_CURRENT_ATTRIB_ID + 1 )

/*
 * fanTable
 */

#define NOS_FAN_INDEX_ATTRIB_ID                       ( NOS_AIRCON_POWER_ATTRIB_ID + 1 )
#define NOS_FAN_STATUS_ATTRIB_ID                      ( NOS_FAN_INDEX_ATTRIB_ID + 1 )
#define NOS_FAN_VOLTAGE_ATTRIB_ID                     ( NOS_FAN_STATUS_ATTRIB_ID + 1 )
#define NOS_FAN_CURRENT_ATTRIB_ID                     ( NOS_FAN_VOLTAGE_ATTRIB_ID + 1 )
#define NOS_FAN_POWER_ATTRIB_ID                       ( NOS_FAN_CURRENT_ATTRIB_ID + 1 )


/*
 * genTable
 */
#define NOS_ATS_INDEX_ATTRIB_ID                       ( NOS_FAN_POWER_ATTRIB_ID + 1 )
#define NOS_ATS_NAME_ATTRIB_ID                        ( NOS_ATS_INDEX_ATTRIB_ID + 1 )
#define NOS_ATS_VALUE_ATTRIB_ID                       ( NOS_ATS_NAME_ATTRIB_ID + 1 )
#define NOS_ATS_STATUS_ATTRIB_ID                      ( NOS_ATS_VALUE_ATTRIB_ID + 1 )
#define NOS_ATS_AUTO_MANUAL_ATTRIB_ID                 ( NOS_ATS_STATUS_ATTRIB_ID + 1 )
#define NOS_ATS_ENABLE_STATUS_ATTRIB_ID               ( NOS_ATS_AUTO_MANUAL_ATTRIB_ID + 1 )
#define NOS_ATS_GRID_STATUS_ATTRIB_ID                 ( NOS_ATS_ENABLE_STATUS_ATTRIB_ID + 1 )
#define NOS_ATS_DG_STATUS_ATTRIB_ID                   ( NOS_ATS_GRID_STATUS_ATTRIB_ID + 1 )
#define NOS_ATS_BTS_BATTERY_STATUS_ATTRIB_ID          ( NOS_ATS_DG_STATUS_ATTRIB_ID + 1 )
#define NOS_ATS_OUTPUT_ATTRIB_ID                      ( NOS_ATS_BTS_BATTERY_STATUS_ATTRIB_ID + 1 )
#define NOS_ATS_DG_CONTROL_ATTRIB_ID                  ( NOS_ATS_OUTPUT_ATTRIB_ID + 1 )
#define NOS_ATS_CONTROL_ATTRIB_ID                     ( NOS_ATS_DG_CONTROL_ATTRIB_ID + 1 )
#endif

