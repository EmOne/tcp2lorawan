//------------------------------------------------------------------------------
//
//	File:		WMDefs.h
//
//	Abstract:	Basic Type and Macro Definitions
//
//	Version:	0.1
//
//	Date:		14.05.2014
//
//	Disclaimer:	This example code is provided by IMST GmbH on an "AS IS" basis
//				without any warranties.
//
//------------------------------------------------------------------------------
//@brief Basic Type and Macro Definitions
//------------------------------------------------------------------------------
//! @cond Doxygen_Suppress



#ifndef WMDEFS_H
#define WMDEFS_H

#include <stdint.h>
#include <stdbool.h>

// enable the next line for C++11 support
//#define WIMOD_USE_CPP11

/*
 * The user has to use the .Process() function at regular base in oder to
 * process the incomming messages on the serial interface bewtween the host
 * (Arduino) and the WiMOD radio module.
 * If the frequency of the calling of the Process() function is too low there
 * is a certain risk of missing / mixing messages.
 * Using a 2nd RxBuffer will reduce this risk alot, but costs more RAM memory
 */
#define WIMOD_USE_2ND_RXBUFFER


typedef uint8_t     UINT8;
typedef uint16_t    UINT16;
typedef uint32_t    UINT32;
typedef uint64_t    UINT64;

typedef int8_t      INT8;
typedef int16_t     INT16;
typedef int32_t     INT32;


#ifndef  MAKEWORD
    #define MAKEWORD(lo,hi)     (UINT16)(((UINT16)((UINT8)(hi)) << 8) | ((UINT16)((UINT8)(lo))))
#endif

#ifndef  MAKELONG
    #define MAKELONG(lo,hi)     (UINT32)(((UINT32)((UINT16)(hi)) << 16) | ((UINT32)((UINT16)(lo))))
#endif

#ifndef LOBYTE
    #define LOBYTE(w)           (UINT8)((UINT16)(w))
#endif

#ifndef HIBYTE
    #define HIBYTE(w)           (UINT8)((UINT16)(w) >> 8)
#endif

#ifndef LOWORD
    #define LOWORD(w)           (UINT16)((UINT32)(w))
#endif

#ifndef HIWORD
    #define HIWORD(w)           (UINT16)((UINT32)(w) >> 16)
#endif




static inline UINT16
NTOH16(const UINT8* srcPtr)
{
    UINT16 value;

    value = MAKEWORD(srcPtr[0], srcPtr[1]);

    return value;
}

static inline void
HTON16(UINT8* dstPtr, UINT16 value)
{
    dstPtr[0] = LOBYTE(value);
    dstPtr[1] = HIBYTE(value);
}

static inline UINT32
NTOH24(const UINT8* srcPtr)
{
    UINT32 value;

    value = MAKELONG(MAKEWORD(srcPtr[0], srcPtr[1]),
            MAKEWORD(srcPtr[2], 0x00));

    return value;
}

static inline void
HTON24(UINT8* dstPtr, UINT32 value)
{
    dstPtr[0] = LOBYTE(LOWORD(value));
    dstPtr[1] = HIBYTE(LOWORD(value));
    dstPtr[2] = LOBYTE(HIWORD(value));
}

static inline UINT32
NTOH32(const UINT8* srcPtr)
{
    UINT32 value;

    value = MAKELONG(MAKEWORD(srcPtr[0], srcPtr[1]),
            MAKEWORD(srcPtr[2], srcPtr[3]));

    return value;
}

static inline void
HTON32(UINT8* dstPtr, UINT32 value)
{
    dstPtr[0] = LOBYTE(LOWORD(value));
    dstPtr[1] = HIBYTE(LOWORD(value));
    dstPtr[2] = LOBYTE(HIWORD(value));
    dstPtr[3] = HIBYTE(HIWORD(value));
}


#ifndef MIN
  #define MIN(a, b) ((a) < (b) ? (a) : (b))
  #define MAX(a, b) ((a) < (b) ? (b) : (a))
#endif

#endif // WMDEFS_H

//! @endcond

//------------------------------------------------------------------------------
// end of file
//------------------------------------------------------------------------------
