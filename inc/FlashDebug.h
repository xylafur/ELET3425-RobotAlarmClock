#ifndef FLASHDEBUG_HEADER
#define FLASHDEBUG_HEADER

#include <stdint.h>

/*  Clears out the region of flash we can write to for us (does this by writing
 *  all 1s to it)
 */
void Debug_FlashInit(void);

/*  Writes a single flash block to flash.  The size of this flash block is
 *  defined in the c file.
 */
void Debug_FlashRecord(uint16_t *pt);

/*  Wrapper for Debug_FlashRecord
 *
 *  Takes in a single 16 bit piece of data and will write it to a buffer.  Once
 *  that buffer is the size of a page in flash then the buffer will be written
 *  to the next valid spot in flash
 */
uint32_t buffer_write_flash_flush(uint16_t data);
uint32_t write_flash_force_flush(uint16_t data);

#endif
