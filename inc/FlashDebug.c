#include <stdint.h>
#include "FlashProgram.h"

#define FLASH_TOP 0x00020000
#define FLASH_BOT 0x0003FFFF

#define FLASH_BLOCK_SIZE 512
#define _4K (4 * 1024)

static void *flash_ptr;

void Debug_FlashInit(void){
    //we can erase flash as 1 4k block at a time

    for(flash_ptr = (void*)FLASH_TOP ; flash_ptr < FLASH_BOT; flash_ptr += _4K){
        Flash_Erase((uint32_t)flash_ptr);
    }
    flash_ptr = (void*)FLASH_TOP;
}

#define BYTES_PER_BIT(n) (n / 8)

void Debug_FlashRecord(uint16_t *pt){
    uint8_t ii;

    //TODO: REmove me!
    if(flash_ptr >= FLASH_BOT)
        return;

    //go through and write 32 bytes at a time
    for(ii=0; ii<(FLASH_BLOCK_SIZE/BYTES_PER_BIT(32)); ii++){
        Flash_Write(flash_ptr, ((uint32_t*)pt)[ii]);
        flash_ptr += BYTES_PER_BIT(32);
    }
    if(flash_ptr >= FLASH_BOT){
        flash_ptr = FLASH_TOP;
    }
}

uint16_t Buffer[FLASH_BLOCK_SIZE];
uint16_t buffer_pos = 0;

void buffer_write_flash_flush(uint16_t data){
    if(buffer_pos == 0){
        Buffer[buffer_pos++] = 0xdead;
        Buffer[buffer_pos++] = 0xbeef;
    }

    Buffer[buffer_pos++] = data;
    if(buffer_pos >= FLASH_BLOCK_SIZE){
        Debug_FlashRecord(Buffer);
        buffer_pos = 0;
        memset(Buffer, 0, FLASH_BLOCK_SIZE);
    }
}


void write_flash_force_flush(uint16_t data){
    if(buffer_pos != 0){
        Debug_FlashRecord(Buffer);
        memset(Buffer, 0, FLASH_BLOCK_SIZE);
        buffer_pos = 0;
    }
    Buffer[buffer_pos] = data;
    Debug_FlashRecord(Buffer);
    memset(Buffer, 0, FLASH_BLOCK_SIZE);
    buffer_pos = 0;
}

/*  I don't think that I'll ever need these, but Just in case I'm leaving this
 *  here
 *
static uint16_t ram_dump_data [NUM_RAM_DUMP_ENTRIES];
static uint8_t ram_dump_index;

void Debug_Init(void){
    memset(ram_dump_data, 0, NUM_RAM_DUMP_ENTRIES * sizeof(uint16_t));;
    ram_dump_index = 0;

}
void Debug_Dump(uint8_t bump, uint8_t line){
  // write this as part of Lab 10
  ram_dump_data[ram_dump_index++] = (bump << 8) | line;

}
 *
 */

