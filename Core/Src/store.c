#include "store.h"
#include "main.h"
#include <stddef.h>
#include <string.h>

#define BLANK         0xFFFFFFFFu
#define SLOT_ADDR(i)  (STORE_BASE + (uint32_t)(i) * SLOT_SIZE)
#define SLOT_HEAD(i)  ((const slot_head_t *)SLOT_ADDR(i))

static uint32_t crc32_calc(const uint8_t *p, uint32_t len)
{
    uint32_t c = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < len; i++) {
        c ^= p[i];
        for (uint8_t k = 0; k < 8; k++) {
            c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
        }
    }
    return ~c;
}

static void cache_flush(void)
{
    __HAL_FLASH_DATA_CACHE_DISABLE();
    __HAL_FLASH_DATA_CACHE_RESET();
    __HAL_FLASH_DATA_CACHE_ENABLE();
}

static bool flash_write(uint32_t addr, const uint32_t *src, uint32_t words)
{
    HAL_StatusTypeDef st = HAL_OK;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    for (uint32_t i = 0; i < words && st == HAL_OK; i++) {
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 4u * i, src[i]);
    }
    HAL_FLASH_Lock();
    cache_flush();
    return st == HAL_OK;
}

static bool slot_blank(uint8_t slot)
{
    const uint32_t *p = (const uint32_t *)SLOT_ADDR(slot);
    for (uint32_t i = 0; i < SLOT_SIZE / 4u; i++) {
        if (p[i] != BLANK) return false;
    }
    return true;
}

void Store_Init(void)
{
    cache_flush();
}

bool Store_Valid(uint8_t slot)
{
    if (slot >= STORE_SLOTS) return false;
    const slot_head_t *h = SLOT_HEAD(slot);
    return h->magic == STORE_MAGIC && h->n >= 1u && h->n <= REC_MAX;
}

const slot_head_t *Store_Head(uint8_t slot)
{
    return SLOT_HEAD(slot);
}

const uint8_t *Store_Data(uint8_t slot)
{
    return (const uint8_t *)(SLOT_ADDR(slot) + sizeof(slot_head_t));
}

uint8_t Store_Used(void)
{
    uint8_t used = 0;
    for (uint8_t i = 0; i < STORE_SLOTS; i++) {
        if (!slot_blank(i)) used++;
    }
    return used;
}

uint8_t Store_Pending(void)
{
    uint8_t pending = 0;
    for (uint8_t i = 0; i < STORE_SLOTS; i++) {
        if (Store_Valid(i) && SLOT_HEAD(i)->dumped != 0u) pending++;
    }
    return pending;
}

uint8_t Store_Free(void)
{
    for (uint8_t i = 0; i < STORE_SLOTS; i++) {
        if (slot_blank(i)) return i;
    }
    return NO_SLOT;
}

bool Store_Save(const uint8_t *buf, uint16_t n, uint8_t label,
                uint32_t t_start, uint32_t t_end)
{
    uint8_t slot = Store_Free();
    if (slot == NO_SLOT || n == 0u || n > REC_MAX) return false;

    uint32_t bytes = (uint32_t)n * 3u;
    uint32_t addr  = SLOT_ADDR(slot);
    uint32_t crc   = crc32_calc(buf, bytes);

    uint32_t h[8];
    memset(h, 0xFF, sizeof h);
    h[1] = t_start;
    h[2] = t_end;
    h[3] = crc;
    h[4] = (uint32_t)n | ((uint32_t)label << 16);

    // samples first, header after, magic dead last
    if (!flash_write(addr + sizeof(slot_head_t), (const uint32_t *)buf, (bytes + 3u) / 4u)) return false;
    if (!flash_write(addr + 4u, &h[1], 4)) return false;
    uint32_t magic = STORE_MAGIC;
    if (!flash_write(addr, &magic, 1)) return false;

    return crc32_calc(Store_Data(slot), bytes) == crc;
}

void Store_MarkDumped(uint8_t slot)
{
    if (!Store_Valid(slot)) return;
    if (SLOT_HEAD(slot)->dumped == 0u) return;
    uint32_t zero = 0u;
    flash_write(SLOT_ADDR(slot) + offsetof(slot_head_t, dumped), &zero, 1);
}

bool Store_Erase(void)
{
    FLASH_EraseInitTypeDef e;
    uint32_t bad = 0;
    HAL_StatusTypeDef st;

    memset(&e, 0, sizeof e);
    e.TypeErase    = FLASH_TYPEERASE_SECTORS;
    e.Sector       = FLASH_SECTOR_7;
    e.NbSectors    = 1;
    e.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    HAL_FLASH_Unlock();
    st = HAL_FLASHEx_Erase(&e, &bad);
    HAL_FLASH_Lock();
    cache_flush();
    return st == HAL_OK;
}
