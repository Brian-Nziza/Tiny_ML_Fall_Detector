#include "model.h"

// placeholder until the trained net is in (x-cube-ai / tflm)
// weak so the generated model can replace these without touching rtos.c

__attribute__((weak)) void Model_Init(void)
{
}

// returns fall probability 0..1, for now trusts the trigger
__attribute__((weak)) float Model_Run(const int8_t in[MODEL_WIN][MODEL_AXES])
{
    (void)in;
    return 1.0f;
}
