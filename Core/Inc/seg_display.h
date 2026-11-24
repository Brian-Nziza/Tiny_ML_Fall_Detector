#ifndef SEG_DISPLAY_H
#define SEG_DISPLAY_H

#include <stdint.h>

void Disp_Init(void);
void Disp_Tick(void);
void Disp_SetNumber(uint16_t n);
void Disp_ShowErr(void);
void Disp_ShowDashes(void);
void Disp_ShowWord(const char *s);

#endif
