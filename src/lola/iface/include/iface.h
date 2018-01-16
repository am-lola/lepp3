/*!
@file iface.h

 definitions common to all/several interfaces

 Copyright (C)  Institute of Applied Mechanics, TU-Muenchen
 All rights reserved.


*/
#ifndef __IFACE_H__
#define __IFACE_H__



/* /\************************************************** */
/*  make sure we have c99 integer types */
/* **************************************************\/ */
/* #if (defined WIN32)||(defined WIN64) */
/* typedef unsigned char uint8_t; */
/* typedef unsigned int uint32_t; */
/* typedef unsigned __int64 uint64_t; */
/* #else */
/*    #include <stdint.h> */
/* #endif */


/**************************************************
 plattform independent structure packing/alignment
**************************************************/
/* #define BEGIN_IFACE _Pragma("pack(1)")  */
//?? swig doesn't understand this??
/* #ifdef __cplusplus */
/* #define BEGIN_NAMESPACE(x) namespace x{ */
/* #define END_NAMESPACE } */
/* #else */
/* #define BEGIN_NAMESPACE(x) */
/* #define END_NAMESPACE */
/* #endif */

#define DATA_VALID   0xAAAAFF00
//hopefully uninitialized data will be all zero ;)
#define DATA_INVALID 0x00000000

#endif//__IFACE_H__
