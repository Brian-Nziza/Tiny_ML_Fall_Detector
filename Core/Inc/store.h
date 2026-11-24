#ifndef STORE_H
#define STORE_H

#include <stdint.h>
#include <stdbool.h>

#define STORE_BASE   0x08060000u
#define STORE_SLOTS  10u
#define SLOT_SIZE    12288u
#define REC_MAX      4000u
#define STORE_MAGIC  0xFA11DA7Au
#define NO_SLOT      0xFFu

typedef struct {
    uint32_t magic;
    uint32_t t_start;
    uint32_t t_end;
    uint32_t crc;
    uint16_t n;
    uint8_t  label;
    uint8_t  pad;
    uint32_t dumped;
    uint32_t spare[2];
} slot_head_t;

_Static_assert(sizeof(slot_head_t) == 32, "header must be 8 words");
_Static_assert(sizeof(slot_head_t) + REC_MAX * 3u <= SLOT_SIZE, "recording does not fit in a slot");
_Static_assert(STORE_SLOTS * SLOT_SIZE <= 0x20000u, "slots do not fit in sector 7");

void           Store_Init(void);
uint8_t        Store_Used(void);
uint8_t        Store_Pending(void);
uint8_t        Store_Free(void);
bool           Store_Valid(uint8_t slot);
const slot_head_t *Store_Head(uint8_t slot);
const uint8_t *Store_Data(uint8_t slot);
bool           Store_Save(const uint8_t *buf, uint16_t n, uint8_t label,
                          uint32_t t_start, uint32_t t_end);
void           Store_MarkDumped(uint8_t slot);
bool           Store_Erase(void);

#endif
