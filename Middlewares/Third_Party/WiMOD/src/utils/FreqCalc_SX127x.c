//------------------------------------------------------------------------------
//! @file FreqCalc_SX127x.c
//! @ingroup Utils
//! <!------------------------------------------------------------------------->
//! @brief Helper Utility to calc frequency to transceiver register and vice versa
//! @version 0.1
//! <!------------------------------------------------------------------------->
//!
//! Implementation of frequency calculator
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

#include "FreqCalc_SX127x.h"

/**
 * @brief Translates a given frequency to the according register values of the TRX
 *
 * Note: This is not 100% exact. (see formula in TRX datasheet)
 *
 * @param freq	RF mid frequency in Hz
 * @param msb	pointer where to store the MSB register value
 * @param mid	pointer where to store the MID register value
 * @param lsb	pointer where to store the LSB register value
 */
void FreqCalc_calcFreqToRegister(uint32_t freq, uint8_t* msb, uint8_t* mid, uint8_t* lsb)
{
	uint64_t frf;

	if (msb && mid && lsb) {
		frf = ((uint64_t) freq << 19) / 32000000;
		*msb = (uint8_t)(frf>>16);
		*mid = (uint8_t)(frf>> 8);
		*lsb = (uint8_t)(frf>> 0);
	}
}

/**
 * @brief Translates given register values to a carrier frequency
 *
 * Note: This is not 100% exact. (see formula in TRX datasheet) *
 *
 * @param msb	the MSB register value
 * @param mid	the MID register value
 * @param lsb	the LSB register value
 *
 * @retval the corresponding frequency in Hz
 */
uint32_t FreqCalc_calcRegisterToFreq(uint8_t msb, uint8_t mid, uint8_t lsb) {
	uint64_t frf;

	frf = 0;
	frf =  lsb;
	frf |= (mid << 8);
	frf |= (msb << 16);
	frf *= 32000000;
	frf >>= 19;
	return (uint32_t) frf;
}
