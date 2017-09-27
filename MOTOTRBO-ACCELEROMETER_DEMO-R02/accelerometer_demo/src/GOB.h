#ifndef GOB_H_
#define GOB_H_

#include <avr32/io.h>
#include "compiler.h"

#define FOSC32          32768                                 //!< Osc32 frequency: Hz.
#define OSC32_STARTUP   AVR32_PM_OSCCTRL32_STARTUP_8192_RCOSC //!< Osc32 startup time: RCOsc periods.

#define FOSC0           12000000                              //!< Osc0 frequency: Hz.
#define OSC0_STARTUP    AVR32_PM_OSCCTRL0_STARTUP_2048_RCOSC  //!< Osc0 startup time: RCOsc periods.


//Timer mappings for GOB.
#define TC_SSI_TRISTATE_CHANNEL         0

#define TC_SSI_CLK_PIN            AVR32_TC_CLK0_0_1_PIN
#define TC_SSI_CLK_FUNCTION       AVR32_TC_CLK0_0_1_FUNCTION
#define TC_SSI_FS_PIN             AVR32_TC_B0_0_0_PIN
#define TC_SSI_FS_FUNCTION        AVR32_TC_B0_0_0_FUNCTION
#define TC_SSI_EN_PIN             AVR32_TC_A0_0_0_PIN
#define TC_SSI_EN_FUNCTION        AVR32_TC_A0_0_0_FUNCTION


#define PDCA_CHANNEL_SSCRX_EXAMPLE 0
#define PDCA_CHANNEL_SSCTX_EXAMPLE 1

#endif /*GOB_H_*/
