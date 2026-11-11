#ifndef LIS35DE_H
#define LIS35DE_H

#include <stdint.h>
#include <stdbool.h>

/* Raw 8-bit two's-complement readings. At FS=1 (+-8 g) one count is ~72 mg. */
typedef struct {
    int8_t x, y, z;
} lis_sample_t;

/* Powers up the sensor: 400 Hz, +-8 g, X/Y/Z enabled.
 * Returns true if WHO_AM_I read back 0x3B. The value actually read is
 * stored in *who (useful for the error message if it is wrong). */
bool Lis_Init(uint8_t *who);

/* True when STATUS_REG says a fresh X/Y/Z triple is waiting (ZYXDA bit). */
bool Lis_DataReady(void);

/* Reads OUT_X, OUT_Y, OUT_Z. Reading OUT_* clears the data-ready flag. */
void Lis_ReadXYZ(lis_sample_t *s);

#endif /* LIS35DE_H */
