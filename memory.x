MEMORY
{
  /* FLASH and RAM are mandatory memory regions */

  /* STM32H503 */
  FLASH   : ORIGIN = 0x08000000, LENGTH = 128K
  /* SRAM1 and SRAM2 are contiguous and can be combined into a single memory area */
  /* SRAM1   : ORIGIN = 0x20000000, LENGTH = 16K */
  /* SRAM2   : ORIGIN = 0x20004000, LENGTH = 16K */
  RAM     : ORIGIN = 0x20000000, LENGTH = 32K
  BKPSRAM : ORIGIN = 0x40036400, LENGTH = 2K
}

/* The location of the stack can be overridden using the
   `_stack_start` symbol.  Place the stack at the end of RAM */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

/* The location of the .text section can be overridden using the
   `_stext` symbol.  By default it will place after .vector_table */
/* _stext = ORIGIN(FLASH) + 0x40c; */
