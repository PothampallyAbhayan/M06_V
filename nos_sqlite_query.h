/*
 ****************************************************************************************
 * nos_sqlite_query.h
 *
 * Author    : Infinicomm Technologies
 * Ver       : 1.0
 * Date      : 03-Jan-2019
 *
 * Copyright Infinicomm Technologies Ltd, 2019-20
 *
 ****************************************************************************************
 */

#ifndef _NOS_SQLITE_QUERY_H_
#define _NOS_SQLITE_QUERY_H_
/*
 * All Select queries are defined here.
 */
#define NOS_DATA_TABLE_SELECT_STR                      "SELECT * from object_data_table " \
                                                       "where  " \
                                                       "object_class=%d AND " \
                                                       "object_sub_class=%d AND " \
                                                       "attribute_id=%d ;"

#define NOS_DATA_TABLE_MATCH_ALL_SELECT_STR            "SELECT * from object_data_table " \
                                                       "where  " \
                                                       "object_index=%d AND " \
                                                       "object_class=%d AND " \
                                                       "object_sub_class=%d AND " \
                                                       "attribute_id=%d ;"


#define NOS_DATA_TABLE_ORDER_SELECT_STR                "SELECT * from object_data_table " \
                                                       "where  " \
                                                       "attribute_id BETWEEN %d AND %d AND " \
                                                       "object_class=%d AND " \
                                                       "object_sub_class=%d " \
                                                       "ORDERED BY attribute_id ;"

#define NOS_DATA_TABLE_TRAP_SELECT_STR                 "SELECT * FROM object_data_table " \
                                                       "where  "\
                                                       "attribute_id BETWEEN %d AND %d   "\
                                                       "AND object_class=%d AND object_sub_class=%d   "\
                                                       "ORDER BY attribute_id ;"


#define NOS_DESCRIPTION_TABLE_SELECT_STR               "SELECT * from object_description_table " \
                                                       "where  " \
                                                       "object_class=%d AND " \
                                                       "object_sub_class=%d AND " \
                                                       "attribute_id=%d AND object_index = %d ;"


#define NOS_DATA_TABLE_SENSOR_SELECT_STR               "SELECT * from object_data_table " \
                                                       "where  " \
                                                       "object_class=%d AND " \
                                                       "object_sub_class=%d AND " \
                                                       "attribute_id=%d AND object_index = %d ;"

#define NOS_DESCRIPTION_TABLE_SELECT_REL_STR           "SELECT * from object_data_table " \
                                                       "where  " \
                                                       "object_class=%d AND " \
                                                       "object_sub_class=%d AND " \
                                                       "attribute_id=%d AND object_index = %d " \
                                                       "data_value = %s ;"

#define NOS_DATA_TABLE_SENSOR_GENERIC_SELECT_STR       "SELECT * from object_data_table " \
                                                       "where  " \
                                                       "object_class=%d AND " \
                                                       "(object_sub_class=%d OR " \
                                                       "object_sub_class=%d OR " \
                                                       "object_sub_class=%d OR " \
                                                       "object_sub_class=%d OR " \
                                                       "object_sub_class=%d) AND " \
                                                       "attribute_id=%d;"

#define NOS_DATA_TABLE_SENSOR_MATCH_ALL_SELECT_STR     "SELECT * from object_data_table " \
                                                       "where  " \
                                                       "object_index=%d AND " \
                                                       "object_class=%d AND " \
                                                       "(object_sub_class=%d OR " \
                                                       "object_sub_class=%d OR " \
                                                       "object_sub_class=%d OR " \
                                                       "object_sub_class=%d OR " \
                                                       "object_sub_class=%d) AND " \
                                                       "attribute_id=%d;"

#define NOS_DATA_TABLE_POWER_SOURCE_SELECT_STR    "SELECT * from object_data_table " \
                                                       "where  " \
                                                       "object_class=%d AND " \
                                                       "object_sub_class=%d AND " \
                                                       "attribute_id=%d AND data_value = '%s' ;"

/*
 * All Insert queries are defined here.
 */
#define NOS_DESCRIPTION_TABLE_INSERT_STR               "INSERT INTO " \
                                                       "object_description_table " \
                                                       "VALUES(%d, %d, %d, '%s', %d );"
 
#define NOS_DATA_TABLE_INSERT_STR                      "INSERT INTO " \
                                                       "object_data_table " \
                                                        "VALUES(%d, %d, %d, %d, %d, '%s' );"


/*
 * All Update queries are defined here.
 */
#define NOS_DATA_TABLE_UPDATE_STR                      "UPDATE "\
                                                       "object_data_table " \
                                                       "SET " \
                                                       "object_index=%d, " \
                                                       "object_class=%d, " \
                                                       "object_sub_class=%d, " \
                                                       "attribute_id=%d, " \
                                                       "data_type=%d, " \
                                                       "data_value='%s' " \
                                                       "WHERE " \
                                                       "object_index=%d AND " \
                                                       "object_class=%d AND " \
                                                       "object_sub_class=%d AND " \
                                                       "attribute_id=%d ; "



#endif

