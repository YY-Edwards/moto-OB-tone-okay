/*
 * my_INTC.c
 *
 *  Created on: May 6, 2009
 *      Author: cmr003
 */

#include <avr32/io.h>
#include "compiler.h"
#include "preprocessor.h"
#include "my_INTC.h"


//Values to store in the interrupt priority registers for the
//various interrupt priority levels.
extern const U32 ipr_val[4];  //These are the  glue routines provided by exception.S
extern const __int_handler interrupt_priority_handlers[4]; //This is my fixed registration.
extern const int priorityMapping[AVR32_INTC_NUM_INT_GRPS]; //This is my fixed priority mapping.




//Gets the interrupt handler of the current int_lev
__int_handler _get_interrupt_handler(unsigned int int_lev)
{
	return interrupt_priority_handlers[int_lev];
}

void my_init_interrupts(void)
{
  unsigned int int_grp;
  int requestedPriority;

  // For all interrupt groups,
  for (int_grp = 0; int_grp < AVR32_INTC_NUM_INT_GRPS; int_grp++)
  {
    requestedPriority = priorityMapping[int_grp];
    if (requestedPriority >= 0)
    {
    	AVR32_INTC.ipr[int_grp] = ipr_val[requestedPriority];
    }
    else
    {
    	AVR32_INTC.ipr[int_grp] = 0x00000000; //Zero Offset from _evba is unrecoverable.
    }
  }
}

