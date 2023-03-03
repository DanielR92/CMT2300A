#ifndef _TYPEDEFS_H_
#define _TYPEDEFS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <Arduino.h>

#define xdata
#define data
#define code const

#ifndef u8
#define u8  uint8_t
#endif

#ifndef u16
#define u16 uint16_t
#endif

#ifndef u32
#define u32 uint32_t
#endif

#ifndef U8
#define U8  uint8_t
#endif

#ifndef S8
#define S8  int8_t
#endif

#ifndef U16
#define U16 uint16_t
#endif

#ifndef U32
#define U32 uint32_t
#endif

#ifndef BOOL
#define BOOL bool
#endif

#ifndef TRUE
#define TRUE true
#endif

#ifndef FALSE
#define FALSE false
#endif

#define INFINITE 0xFFFFFFFF

#ifdef __cplusplus
}
#endif

#endif
