#ifndef SEG_DISPLAY_H
#define SEG_DISPLAY_H

#include <stdint.h>

/* 4-digit multiplexed 7-segment display on the KA-Nucleo-Multisensor shield.
 * Segments A..G,DP = PC6 PC4 PC3 PC1 PC2 PC5 PC0 PC7
 * Digit commons    = PC11 PC10 PC9 PC8 (digit 0 = left) */

void Disp_Init(void);                 /* configures PC0..PC11 as outputs      */
void Disp_Tick(void);                 /* call at 1 kHz from a timer interrupt */
void Disp_SetNumber(uint16_t n);      /* shows 0..9999, e.g. " 001"           */
void Disp_ShowErr(void);              /* shows "Err" (sensor not found)       */
void Disp_ShowDashes(void);           /* shows "----" (booting)               */

#endif /* SEG_DISPLAY_H */
