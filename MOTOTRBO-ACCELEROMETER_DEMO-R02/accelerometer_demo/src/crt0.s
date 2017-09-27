
//! @{
//! \verbatim


  // This must be linked @ 0x80000000 if it is to be run upon reset.
  .section  .reset, "ax", @progbits


  .global _start
  .type _start, @function
_start:
  // Jump to the C runtime startup routine.
  lda.w   pc, _stext



  // _stext is placed outside the .reset section so that the program entry point
  // can be changed without affecting the C runtime startup.
  .section  .text._stext, "ax", @progbits


  .global _stext
  .type _stext, @function
_stext:
  // Set initial stack pointer.
  lda.w   sp, _estack

  // Set up EVBA so interrupts can be enabled.
  lda.w   r0, _evba
  mtsr    0x00000004, r0

  // Enable the exception processing.
  csrf    21

  // Load initialized data having a global lifetime from the data LMA.
  lda.w   r0, _data
  lda.w   r1, _edata
  cp      r0, r1
  brhs    idata_load_loop_end
  lda.w   r2, _data_lma
idata_load_loop:
  ld.d    r4, r2++
  st.d    r0++, r4
  cp      r0, r1
  brlo    idata_load_loop
idata_load_loop_end:

  // Clear uninitialized data having a global lifetime in the blank static storage section.
  lda.w   r0, __bss_start
  lda.w   r1, _end
  cp      r0, r1
  brhs    udata_clear_loop_end
  mov     r2, 0
  mov     r3, 0
udata_clear_loop:
  st.d    r0++, r2
  cp      r0, r1
  brlo    udata_clear_loop
udata_clear_loop_end:


  // Safety: Set the default "return" @ to the exit routine address.
  //lda.w   lr, exit
  //For real time code, exit returns to start.
  //lda.w   lr, _stext


  // Start the show.
  lda.w   pc, main


//! \endverbatim
//! @}
