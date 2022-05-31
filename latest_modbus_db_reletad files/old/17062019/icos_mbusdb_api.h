/*
 ****************************************************************************************
 * icos_mbusdb_api.h
 *
 * Author    : Infinicomm Technologies
 * Ver       : 1.0
 * Date      : 03-Jan-2019
 *
 * Copyright Infinicomm Technologies Ltd, 2019-20
 *
 ****************************************************************************************
 */
#ifndef _ICOS_MBUSDB_API_H_
#define _ICOS_MBUSDB_API_H_

#include "icos_types.h"
#include "nos_db.h"

/*
 * All mbusdb config API declarations are here.
 */

/*
 * Init db handle
 * Return:
 *     db handle
 */
extern NOS_DB_HANDLE* icos_mbusdb_init();

/*
 * Get db handle
 * Return:
 *     db handle
 */
extern NOS_DB_HANDLE* icos_mbusdb_get_db_handle();

/*
 * Create a db entry for a register
 * Parameters:
 *     if index assigned
 *     modbus bus number
 */
extern icos_int16 icos_mbusdb_create_reg_interface( NOS_DB_HANDLE *p_hndl, icos_uint32 if_index_cur, icos_int16 modbus_bus_no );

/*
 * Update register data in db
 * Parameters:
 *     if index assigned for the register
 *     modbus bus number
 *     slave id of modbus client
 *     reg address
 *     reg_type
 *     reg_value
 */
extern icos_int16 icos_mbusdb_update_reg( NOS_DB_HANDLE *p_hndl, icos_uint32 if_index, 
                                          icos_int16 bus_no, icos_int16 slave_id,
		                          icos_int32 reg_add, icos_int16 reg_type, 
                                          icos_uint32 reg_value);

/*
 * Read register data from db using if_index
 * Parameters:
 *     db handle
 *     if index assigned for the register
 *     pointer for store data size
 *     pointer for data
 */
extern icos_int32 icos_mbusdb_read_reg_data ( NOS_DB_HANDLE *p_hndl, icos_uint32 if_index, icos_uint32 *data_size, icos_uchar **data);

/*
 * Read register data from db using reg address
 * Parameters:
 *     db handle
 *     modbus bus number
 *     slave id of modbus client
 *     reg address
 *     pointer for store data size
 *     pointer for data
 */
extern icos_int32 icos_mbusdb_read_reg_data_from_address ( NOS_DB_HANDLE *p_hndl, icos_int16 bus_no,
		icos_int16 slave_id, icos_int32 reg_addr, icos_uint32 *data_size, icos_uchar **data);


/*
 * Check whether the reg entry is exists in db
 * Parameters:
 *     db handle
 *     if index assigned for the register
 *
 * Return:
 *     0 - not exist
 *     1 - Exists
 */
extern icos_int32 icos_mbusdb_if_index_exists ( NOS_DB_HANDLE *p_hndl, icos_uint32 if_index);

/*
 * Find if index of a reg
 * Parameters:
 *     db handle
 *     modbus bus number
 *     slave id of modbus client
 *     reg address
 * Return:
 *     -1 - No such reg entry in db
 *     if index of the reg
 */
extern icos_int32 icos_mbusdb_find_if_index ( NOS_DB_HANDLE *p_hndl, icos_int16 bus_no,
		icos_int16 slave_id, icos_int32 reg_addr);

#endif

