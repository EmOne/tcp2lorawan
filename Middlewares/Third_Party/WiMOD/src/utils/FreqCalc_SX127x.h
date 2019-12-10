//------------------------------------------------------------------------------
//! @file FreqCalc_SX127x.c
//! @ingroup Utils
//! <!------------------------------------------------------------------------->
//! @brief Helper Utility to calc frequency to transceiver register and vice versa
//! @version 0.1
//! <!------------------------------------------------------------------------->
//!
//! Declarations for frequency calculator
//!
//! <!--------------------------------------------------------------------------
//! Copyright (c) 2016
//! IMST GmbH
//! Carl-Friedrich Gauss Str. 2-4
//! 47475 Kamp-Lintfort
//! --------------------------------------------------------------------------->
//! @author (FB), IMST
//! <!--------------------------------------------------------------------------
//! Target OS:    independent
//! Target CPU:   independent
//! Compiler:     tbd
//! --------------------------------------------------------------------------->
//! @internal
//! @par Revision History:
//! <PRE>
//! ----------------------------------------------------------------------------
//! Version | Date       | Author | Comment
//! ----------------------------------------------------------------------------
//! 0.1     | 16.09.2016 | FB     | Created
//! </PRE>
//------------------------------------------------------------------------------

#ifndef ARDUINO_UTILS_FREQCALC_SX127X_H_
#define ARDUINO_UTILS_FREQCALC_SX127X_H_


#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void     FreqCalc_calcFreqToRegister(uint32_t freq, uint8_t* msb, uint8_t* mid, uint8_t* lsb);
uint32_t FreqCalc_calcRegisterToFreq(uint8_t msb, uint8_t mid, uint8_t lsb);

#ifdef __cplusplus
}
#endif


#endif /* ARDUINO_UTILS_FREQCALC_SX127X_H_ */
