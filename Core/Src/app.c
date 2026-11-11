/*
 * Fall-recording firmware for NUCLEO-F411RE + KA-Nucleo-Multisensor shield.
 *
 * UART protocol (ASCII, one record per line, 460800 baud, 8N1):
 *   #...            comment / status text (ignored by the Python script)
 *   S,<t_us>,<x>,<y>,<z>   one accelerometer sample. t_us = free-running
 *                          1 MHz timer (wraps every 71.6 min), x/y/z = raw
 *                          int8 counts.
 *   E,<id>,<t_us>   "end of event" button: recording number <id> just ended.
 *   R,<t_us>        "end of session" button: counter went back to 0.
 */
#include "app.h"
#include "main.h"
#include "lis35de.h"
#include "seg_display.h"
#include <stdio.h>

extern UART_HandleTypeDef huart2;   /* USART2 -> ST-LINK virtual COM port    */
extern TIM_HandleTypeDef  htim2;    /* free-running 1 MHz, 32-bit            */
extern TIM_HandleTypeDef  htim4;    /* 1 kHz tick: display scan + buttons    */

/* Buttons on the shield: S1 = PB0, S2 = PB5, both to GND (active low). */
#ifndef BTN_EVENT_Pin
#define BTN_EVENT_Pin        GPIO_PIN_0
#define BTN_EVENT_GPIO_Port  GPIOB
#endif
#ifndef BTN_RESET_Pin
#define BTN_RESET_Pin        GPIO_PIN_5
#define BTN_RESET_GPIO_Port  GPIOB
#endif

#define DEBOUNCE_MS 20u

typedef struct {
    GPIO_TypeDef   *port;
    uint16_t        pin;
    uint8_t         stable;      /* debounced state: 1 = pressed          */
    uint8_t         count;       /* ms the raw level has disagreed        */
    volatile uint8_t pressed;    /* set by ISR on press, cleared by loop  */
} button_t;

static button_t btn_event = { BTN_EVENT_GPIO_Port, BTN_EVENT_Pin, 0, 0, 0 };
static button_t btn_reset = { BTN_RESET_GPIO_Port, BTN_RESET_Pin, 0, 0, 0 };

static uint32_t rec_count = 0;   /* number shown on the display */

/* ---- helpers ------------------------------------------------------------ */

static void uart_write(const char *s, int len)
{
    if (len > 0) {
        HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)len, 20);
    }
}

static void button_tick(button_t *b)           /* runs in the 1 kHz ISR */
{
    uint8_t raw = (HAL_GPIO_ReadPin(b->port, b->pin) == GPIO_PIN_RESET);

    if (raw == b->stable) {
        b->count = 0;
        return;
    }
    if (++b->count >= DEBOUNCE_MS) {           /* level held long enough */
        b->stable = raw;
        b->count  = 0;
        if (raw) b->pressed = 1;               /* count presses only    */
    }
}

/* TIM4 update interrupt, 1 kHz. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {
        Disp_Tick();
        button_tick(&btn_event);
        button_tick(&btn_reset);
    }
}

/* ---- public API --------------------------------------------------------- */

void App_Init(void)
{
    char line[64];
    int  n;
    uint8_t who = 0;

    Disp_Init();                               /* shows "----" */

    /* Start from the pins' current level so a held button at boot
     * is not reported as a press. */
    btn_event.stable = (HAL_GPIO_ReadPin(btn_event.port, btn_event.pin) == GPIO_PIN_RESET);
    btn_reset.stable = (HAL_GPIO_ReadPin(btn_reset.port, btn_reset.pin) == GPIO_PIN_RESET);

    HAL_TIM_Base_Start(&htim2);                /* timestamp counter      */
    HAL_TIM_Base_Start_IT(&htim4);             /* display + buttons      */

    if (!Lis_Init(&who)) {
        Disp_ShowErr();
        for (;;) {                             /* stay here and complain */
            n = snprintf(line, sizeof line,
                         "#ERR,LIS35DE not found,WHO_AM_I=0x%02X (want 0x3B)\n", who);
            uart_write(line, n);
            HAL_Delay(1000);
        }
    }

    Disp_SetNumber(0);
    n = snprintf(line, sizeof line, "#BOOT,LIS35DE ok,400Hz,8g\n");
    uart_write(line, n);
}

void App_Loop(void)
{
    char line[48];
    int  n;

    /* 1) A new accelerometer sample is ready -> read and stream it. */
    if (Lis_DataReady()) {
        uint32_t t = __HAL_TIM_GET_COUNTER(&htim2);
        lis_sample_t s;
        Lis_ReadXYZ(&s);
        n = snprintf(line, sizeof line, "S,%lu,%d,%d,%d\n",
                     (unsigned long)t, s.x, s.y, s.z);
        uart_write(line, n);
    }

    /* 2) "End of recording" button. Handled after the sample above so the
     *    marker lands after every sample taken before the press. */
    if (btn_event.pressed) {
        btn_event.pressed = 0;
        rec_count++;
        Disp_SetNumber((uint16_t)rec_count);
        n = snprintf(line, sizeof line, "E,%lu,%lu\n",
                     (unsigned long)rec_count,
                     (unsigned long)__HAL_TIM_GET_COUNTER(&htim2));
        uart_write(line, n);
    }

    /* 3) "End of session" button: counter and display back to 0. */
    if (btn_reset.pressed) {
        btn_reset.pressed = 0;
        rec_count = 0;
        Disp_SetNumber(0);
        n = snprintf(line, sizeof line, "R,%lu\n",
                     (unsigned long)__HAL_TIM_GET_COUNTER(&htim2));
        uart_write(line, n);
    }
}
