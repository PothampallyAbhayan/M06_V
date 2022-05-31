/*
 ****************************************************************************************
 * nos_string.h
 *
 * Author    : Infinicomm Technologies
 * Ver       : 1.0
 * Date      : 03-Jan-2019
 *
 * Copyright Infinicomm Technologies Ltd, 2019-20
 *
 ****************************************************************************************
 */

#ifndef _NOS_STRING_H_
#define _NOS_STRING_H_

#include "nos_types.h"

nos_char *nos_strcpy(nos_char *dest, const nos_char *src);
nos_char *nos_strncpy(nos_char *dest, const nos_char *src, nos_uint16 n);
nos_int32 nos_strcmp(const nos_char *str1,const nos_char *str2);
nos_int32 nos_strncmp(const nos_char *str1,const nos_char *str2, nos_uint16 n);
nos_int32 nos_atoi ( const nos_char *str );
nos_double nos_atof(const nos_char *str );
nos_int32 nos_rand(void);
nos_long nos_strtol ( const nos_char *str);
nos_int32 nos_scanf(const char *str, const char *format, ...);
nos_int32 nos_sprintf(char *str, const char *format, ...);
nos_int32 nos_strlen (const char *str);
nos_char  *nos_strcat(nos_char *str1,nos_char *str2);
nos_char  *nos_strtok ( nos_char *str, const nos_char *delim );

#endif

