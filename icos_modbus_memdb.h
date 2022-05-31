/*
 ****************************************************************************************
 * icos_modbus_memdb.h
 *
 * Author    : Infinicomm Solutions
 * Ver       : 1.0
 * Date      : 02-Jan-2020
 *
 * Copyright Infinicomm Solutions India Pvt Ltd, 2020
 *
 ****************************************************************************************
 */

#ifndef _ICOS_MODBUS_MEMDB_H_
#define _ICOS_MODBUS_MEMDB_H_

/*
 *
 * The mapped memory of the MODBUS is as explained below.
 *
 *
 *      +-------------------------------------------------+
 *      |                                                 |
 *      |         ICOS_MODBUS_MEM_DB_SUMMARY              |
 *      |                                                 |
 *      +-------------------------------------------------+
 *      |                                                 |
 *      |         ICOS_MBUS_DB_ELEMENT[] for Slave-1      |
 *      |                                                 |
 *      +-------------------------------------------------+
 *      |                                                 |
 *      |         ICOS_MBUS_DB_ELEMENT[] for Slave-2      |
 *      |                                                 |
 *      +-------------------------------------------------+
 *      |                                                 |
 *      |         ICOS_MBUS_DB_ELEMENT[] for Slave-3      |
 *      |                                                 |
 *      +-------------------------------------------------+
 *
 */

typedef struct _icos_modbus_mem_db_summary_
{


} ICOS_MODBUS_MEM_DB_SUMMARY;

#define MODBUS_INDEX_OFFSET                   0x7000
#define MODBUS_MAX_REG_COUNT                  989
#define MODBUS_REG_START_ADDR                 0x800

/*
 * Possible values of reg_type
 */

#define MBUS_REG_TYPE_COIL_STATUS             0x01
#define MBUS_REG_TYPE_INPUT_STATUS            0x02
#define MBUS_REG_TYPE_HOLDING_REG             0x03
#define MBUS_REG_TYPE_INPUT_REG               0x04

typedef struct  __attribute__((__packed__)) _icos_mbus_db_element_
{
    icos_uint32       if_index;
    icos_uint16       reg_addr;
    icos_uchar        bus_no;
    icos_uchar        slave_address;
    icos_uchar        reg_type;
    icos_uint16       reg_value;
} ICOS_MBUS_DB_ELEMENT;

#endif

