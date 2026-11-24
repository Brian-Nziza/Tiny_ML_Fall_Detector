#include "app.h"
#include "main.h"
#include "lis35de.h"
#include "seg_display.h"
#include "store.h"
#include <stdio.h>

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef  htim2;
extern TIM_HandleTypeDef  htim4;

#ifndef BTN_EVENT_Pin
#define BTN_EVENT_Pin        GPIO_PIN_0
#define BTN_EVENT_GPIO_Port  GPIOB
#endif
#ifndef BTN_RESET_Pin
#define BTN_RESET_Pin        GPIO_PIN_5
#define BTN_RESET_GPIO_Port  GPIOB
#endif

#define DEBOUNCE_MS   20u
#define DOUBLE_MS     400u
#define COUNTDOWN_MS  3000u
#define SHOW_MS       1000u
#define HOLD_MS       5000u

typedef enum { ST_IDLE, ST_COUNT, ST_REC } state_t;

typedef struct {
    GPIO_TypeDef    *port;
    uint16_t         pin;
    uint8_t          stable;
    uint8_t          count;
    volatile uint8_t pressed;
    volatile uint32_t press_ms;
} button_t;

static button_t btn_event = { BTN_EVENT_GPIO_Port, BTN_EVENT_Pin, 0, 0, 0, 0 };
static button_t btn_reset = { BTN_RESET_GPIO_Port, BTN_RESET_Pin, 0, 0, 0, 0 };

static uint32_t rec_count = 0;
static uint8_t  live = 0;

static state_t  state = ST_IDLE;
static uint32_t state_ms = 0;
static uint32_t last_press = 0;
static uint32_t show_until = 0;
static uint8_t  label = 1;
static uint8_t  k_wait = 0;
static uint8_t  s2_held = 0;
static uint8_t  s2_fired = 0;

static uint16_t rec_n = 0;
static uint32_t rec_t0 = 0;
static uint32_t rec_t1 = 0;
static uint8_t  rec_buf[REC_MAX * 3u] __attribute__((aligned(4)));

static void uart_write(const char *s, int len)
{
    if (len > 0) {
        HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)len, 20);
    }
}

static void button_tick(button_t *b)
{
    uint8_t raw = (HAL_GPIO_ReadPin(b->port, b->pin) == GPIO_PIN_RESET);

    if (raw == b->stable) {
        b->count = 0;
        return;
    }
    if (++b->count >= DEBOUNCE_MS) {
        b->stable = raw;
        b->count  = 0;
        if (raw) {
            b->press_ms = HAL_GetTick();
            b->pressed = 1;
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {
        Disp_Tick();
        button_tick(&btn_event);
        button_tick(&btn_reset);
    }
}

static void show_idle(void)
{
    Disp_SetNumber(Store_Used());
}

static void send_info(void)
{
    char line[48];
    int n = snprintf(line, sizeof line, "#STORE,%u,%u,%u\n",
                     (unsigned)Store_Used(), (unsigned)STORE_SLOTS, (unsigned)Store_Pending());
    uart_write(line, n);
}

static void dump_slots(uint8_t all)
{
    char line[80];
    int n;
    uint8_t sent = 0;

    for (uint8_t i = 0; i < STORE_SLOTS; i++) {
        if (!Store_Valid(i)) continue;
        const slot_head_t *h = Store_Head(i);
        if (!all && h->dumped == 0u) continue;

        n = snprintf(line, sizeof line, "B,%u,%u,%u,%lu,%lu,%08lX\n",
                     (unsigned)i, (unsigned)h->label, (unsigned)h->n,
                     (unsigned long)h->t_start, (unsigned long)h->t_end,
                     (unsigned long)h->crc);
        uart_write(line, n);
        HAL_UART_Transmit(&huart2, (uint8_t *)Store_Data(i), (uint16_t)(h->n * 3u), 3000);
        uart_write("\n", 1);
        sent++;
    }
    n = snprintf(line, sizeof line, "#DUMPDONE,%u\n", (unsigned)sent);
    uart_write(line, n);
}

// 0 nothing, 1 short press let go, 2 held for 5 s
static uint8_t s2_poll(uint32_t now)
{
    if (btn_reset.pressed) {
        btn_reset.pressed = 0;
        s2_held = 1;
        s2_fired = 0;
    }
    if (!s2_held) return 0;
    if (!s2_fired && (now - btn_reset.press_ms) >= HOLD_MS) {
        s2_fired = 1;
        return 2;
    }
    if (!btn_reset.stable) {
        s2_held = 0;
        return s2_fired ? 0 : 1;
    }
    return 0;
}

static void uart_poll(void)
{
    char line[40];
    int n;

    if (!__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) return;
    uint8_t c = (uint8_t)(huart2.Instance->DR & 0xFFu);

    if (k_wait) {
        k_wait = 0;
        if (c >= '0' && c <= '9') {
            Store_MarkDumped((uint8_t)(c - '0'));
            n = snprintf(line, sizeof line, "#ACK,%c\n", c);
            uart_write(line, n);
        }
        return;
    }
    if (c == 'K') { k_wait = 1; return; }
    if (state != ST_IDLE) return;

    switch (c) {
    case 'L':
        live = 1;
        rec_count = 0;
        btn_event.pressed = 0;
        btn_reset.pressed = 0;
        s2_held = 0;
        Disp_SetNumber(0);
        uart_write("#MODE,live\n", 11);
        break;
    case 'l':
        live = 0;
        btn_event.pressed = 0;
        btn_reset.pressed = 0;
        s2_held = 0;
        show_idle();
        uart_write("#MODE,store\n", 12);
        break;
    case 'D': dump_slots(0); break;
    case 'A': dump_slots(1); break;
    case 'I': send_info();   break;
    case 'X':
        Disp_ShowWord("CLr ");
        n = snprintf(line, sizeof line, "#ERASED,%d\n", Store_Erase() ? 1 : 0);
        uart_write(line, n);
        show_idle();
        break;
    default:
        break;
    }
}

static void live_step(void)
{
    char line[48];
    int  n;

    if (Lis_DataReady()) {
        uint32_t t = __HAL_TIM_GET_COUNTER(&htim2);
        lis_sample_t s;
        Lis_ReadXYZ(&s);
        n = snprintf(line, sizeof line, "S,%lu,%d,%d,%d\n",
                     (unsigned long)t, s.x, s.y, s.z);
        uart_write(line, n);
    }

    if (btn_event.pressed) {
        btn_event.pressed = 0;
        rec_count++;
        Disp_SetNumber((uint16_t)rec_count);
        n = snprintf(line, sizeof line, "E,%lu,%lu\n",
                     (unsigned long)rec_count,
                     (unsigned long)__HAL_TIM_GET_COUNTER(&htim2));
        uart_write(line, n);
    }

    uint8_t s2 = s2_poll(HAL_GetTick());
    if (s2 == 1u) {
        rec_count = 0;
        Disp_SetNumber(0);
        n = snprintf(line, sizeof line, "R,%lu\n",
                     (unsigned long)__HAL_TIM_GET_COUNTER(&htim2));
        uart_write(line, n);
    } else if (s2 == 2u) {
        Disp_ShowWord("dUP ");
        dump_slots(0);
        Disp_SetNumber((uint16_t)rec_count);
    }
}

static void finish_recording(void)
{
    state = ST_IDLE;
    if (rec_n == 0u) {
        show_idle();
        return;
    }
    Disp_ShowWord("SAUE");
    uint8_t ok = Store_Save(rec_buf, rec_n, label, rec_t0, rec_t1);
    Disp_ShowWord(ok ? "donE" : "Err ");
    show_until = HAL_GetTick() + SHOW_MS;
}

static void store_step(void)
{
    char w[8];
    uint32_t now = HAL_GetTick();

    if (state == ST_REC && Lis_DataReady()) {
        uint32_t t = __HAL_TIM_GET_COUNTER(&htim2);
        lis_sample_t s;
        Lis_ReadXYZ(&s);
        if (rec_n == 0u) rec_t0 = t;
        rec_t1 = t;
        rec_buf[rec_n * 3u]      = (uint8_t)s.x;
        rec_buf[rec_n * 3u + 1u] = (uint8_t)s.y;
        rec_buf[rec_n * 3u + 2u] = (uint8_t)s.z;
        rec_n++;
        if (rec_n >= REC_MAX) {
            finish_recording();
            return;
        }
    }

    uint8_t s2 = s2_poll(now);
    if (s2 == 1u && state == ST_IDLE) {
        label = (uint8_t)((label % 9u) + 1u);
        snprintf(w, sizeof w, "Lb %u", (unsigned)label);
        Disp_ShowWord(w);
        show_until = now + SHOW_MS;
    } else if (s2 == 2u && state == ST_IDLE) {
        Disp_ShowWord("dUP ");
        dump_slots(0);
        show_idle();
    }

    if (btn_event.pressed) {
        btn_event.pressed = 0;
        uint32_t t = btn_event.press_ms;
        uint8_t dbl = (last_press != 0u) && ((t - last_press) < DOUBLE_MS);
        last_press = t;

        if (state == ST_IDLE) {
            if (Store_Free() == NO_SLOT) {
                Disp_ShowWord("FULL");
                show_until = now + 1500u;
            } else {
                state = ST_COUNT;
                state_ms = now;
                show_until = 0;
                Disp_ShowWord("   3");
            }
        } else if (state == ST_COUNT) {
            state = ST_IDLE;
            show_idle();
        } else if (dbl) {
            finish_recording();
            return;
        }
    }

    if (state == ST_COUNT) {
        uint32_t el = now - state_ms;
        if (el >= COUNTDOWN_MS) {
            rec_n = 0;
            state = ST_REC;
            Disp_ShowWord("rEC ");
        } else {
            snprintf(w, sizeof w, "   %u", (unsigned)(3u - el / 1000u));
            Disp_ShowWord(w);
        }
    }

    if (state == ST_IDLE && show_until != 0u && now >= show_until) {
        show_until = 0;
        show_idle();
    }
}

void App_Init(void)
{
    char line[64];
    int  n;
    uint8_t who = 0;

    Disp_Init();

    btn_event.stable = (HAL_GPIO_ReadPin(btn_event.port, btn_event.pin) == GPIO_PIN_RESET);
    btn_reset.stable = (HAL_GPIO_ReadPin(btn_reset.port, btn_reset.pin) == GPIO_PIN_RESET);

    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start_IT(&htim4);

    Store_Init();

    if (!Lis_Init(&who)) {
        Disp_ShowErr();
        for (;;) {
            n = snprintf(line, sizeof line,
                         "#ERR,LIS35DE not found,WHO_AM_I=0x%02X (want 0x3B)\n", who);
            uart_write(line, n);
            HAL_Delay(1000);
        }
    }

    show_idle();
    n = snprintf(line, sizeof line, "#BOOT,LIS35DE ok,400Hz,8g\n");
    uart_write(line, n);
    send_info();
}

void App_Loop(void)
{
    uart_poll();
    if (live) {
        live_step();
    } else {
        store_step();
    }
}
