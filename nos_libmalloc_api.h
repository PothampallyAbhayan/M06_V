/*
 ****************************************************************************************
 * nos_malloc.h
 *
 * Author    : Infinicomm Technologies
 * Ver       : 1.0
 * Date      : 03-Jan-2019
 *
 * Copyright Infinicomm Technologies Ltd, 2019-20
 *
 ****************************************************************************************
 */

#ifndef _NOS_MALLOC_H_
#define _NOS_MALLOC_H_

#include "nos_types.h"

/*
 * All API Templates are defined here.
 */

void *nos_malloc(int size);
void nos_free(void *);
void *nos_memset(void *ptr, nos_int32 data, nos_int32 size);
void *nos_memcpy(void *dest, const void *src, nos_int32 n);
nos_int32 nos_memcmp(const void *s1, const void *s2, nos_uint32 n);
void *nos_realloc ( void *ptr, nos_int32 size );

#endif 
