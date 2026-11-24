#ifndef LIS35DE_H
#define LIS35DE_H

#include <stdint.h>
#include <stdbool.h>

// raw counts, ~72 mg each at 8g
typedef struct {
    int8_t x, y, z;
} lis_sample_t;

bool Lis_Init(uint8_t *who);

bool Lis_DataReady(void);

void Lis_ReadXYZ(lis_sample_t *s);

#endif
