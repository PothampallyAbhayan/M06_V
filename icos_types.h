/*
 ****************************************************************************************
 * icos_types.h
 *
 * Author    : Infinicomm Technologies
 * Ver       : 1.0
 * Date      : 03-Jan-2019
 *
 * Copyright Infinicomm Technologies Ltd, 2019-20
 *
 ****************************************************************************************
 */

#ifndef _ICOS_TYPES_H_
#define _ICOS_TYPES_H_

/*
 * Declare all iCOS data types here
 */

typedef unsigned char         icos_uchar;            // 8-bit unsigned
typedef char                  icos_char;             // 8-bit signed
typedef unsigned char         icos_boolean;          // Boolean data type
typedef unsigned short int    icos_uint16;           // 16-bit unsigned
typedef short int             icos_int16;            // 16-bit signed
typedef unsigned int          icos_uint32;           // 32-bit unsigned
typedef int                   icos_int32;            // 32-bit signed
typedef double                icos_double;           // 64-bit
typedef float                 icos_float;            // 32-bit
typedef unsigned int          icos_ip_address;       // 32-bit unsigned for ip address
typedef unsigned short int    icos_port;             // 16-bit port

typedef unsigned char         icos_bool;
typedef long int              icos_long;
typedef unsigned long long    icos_uint64;           // 64-bit unsigned
typedef unsigned long         icos_ulong;

#ifndef NULL
#define NULL                  0
#endif

#ifndef TRUE
#define TRUE                  1
#endif
#ifndef FALSE
#define FALSE                 0
#endif

#endif

