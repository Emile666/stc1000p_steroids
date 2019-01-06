#ifndef _STDINT_H_
#define _STDINT_H_
// The Cosmic compiler is not C99 compatible, therefore define these types here

#define UINT8_MAX  (0xFF)       /* 255U */
#define UINT16_MAX (0xFFFF)     /* 65535U */
#define UINT32_MAX (0xFFFFFFFF) /* 4294967295U */

typedef signed char   int8_t;
typedef unsigned char uint8_t;
typedef unsigned int  uint16_t;
typedef int           int16_t;
typedef unsigned long uint32_t;
typedef long          int32_t;

#endif
