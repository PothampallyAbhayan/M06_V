/*
 ****************************************************************************************
 * nos_types.h
 *
 * Author    : Infinicomm Technologies
 * Ver       : 1.0
 * Date      : 03-Jan-2019
 *
 * Copyright Infinicomm Technologies Ltd, 2019-20
 *
 ****************************************************************************************
 */

#ifndef _NOS_TYPES_H_
#define _NOS_TYPES_H_

/*
 * Declare all nOS data types here
 */

typedef unsigned char         nos_uchar;            // 8-bit unsigned
typedef char                  nos_char;             // 8-bit signed
typedef unsigned char         nos_boolean;          // Boolean data type
typedef unsigned short int    nos_uint16;           // 16-bit unsigned
typedef short int             nos_int16;            // 16-bit signed
typedef unsigned int          nos_uint32;           // 32-bit unsigned
typedef int                   nos_int32;            // 32-bit signed
typedef double                nos_double;           // 64-bit
typedef float                 nos_float;            // 32-bit
typedef unsigned int          nos_ip_address;       // 32-bit unsigned for ip address
typedef unsigned short int    nos_port;             // 16-bit port

typedef unsigned char         nos_bool;
typedef long int              nos_long;
typedef unsigned long long    nos_uint64;           // 64-bit unsigned
typedef unsigned long         nos_ulong;

#ifndef NULL
#define NULL                  0
#endif

#ifndef TRUE
#define TRUE                  1
#endif
#ifndef FALSE
#define FALSE                 0
#endif

#define ICOMM_ENTR_ID               53612

#endif

