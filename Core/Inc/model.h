#ifndef MODEL_H
#define MODEL_H

#include <stdint.h>

// 1 s window @ 400 hz, x y z raw counts, oldest first
#define MODEL_WIN   400u
#define MODEL_AXES  3u

void  Model_Init(void);
float Model_Run(const int8_t in[MODEL_WIN][MODEL_AXES]);

#endif
