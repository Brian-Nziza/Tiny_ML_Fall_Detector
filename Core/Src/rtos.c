#include "rtos.h"

// builds to nothing until freertos is turned on in the .ioc
#if __has_include("FreeRTOS.h")

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "main.h"
#include "lis35de.h"
#include "model.h"
#include <string.h>

extern TIM_HandleTypeDef htim2;

// squared magnitude in counts, 1 g ~ 14 at 8g
#define FREEFALL_MAG2  (6 * 6)
#define IMPACT_MAG2    (35 * 35)
#define FREEFALL_MIN   16u
#define IMPACT_WINDOW  160u
#define POST_IMPACT    (MODEL_WIN / 2u)
#define COOLDOWN       800u
#define QUEUE_LEN      64u
#define MODEL_THRESH   0.5f

#define SENSOR_PERIOD  ((pdMS_TO_TICKS(1) > 0) ? pdMS_TO_TICKS(1) : 1)

typedef struct {
    uint32_t t;
    int8_t   x, y, z;
} sample_t;

typedef enum { WAIT_FF, WAIT_IMPACT, WAIT_POST, COOL } det_t;

static QueueHandle_t     sample_q;
static volatile uint32_t dropped;

static int8_t   ring[MODEL_WIN][MODEL_AXES];
static int8_t   win[MODEL_WIN][MODEL_AXES];
static uint16_t head;
static uint16_t filled;

static void sensor_task(void *arg)
{
    (void)arg;
    lis_sample_t s;
    sample_t     q;
    TickType_t   wake = xTaskGetTickCount();

    for (;;) {
        if (Lis_DataReady()) {
            Lis_ReadXYZ(&s);
            q.t = __HAL_TIM_GET_COUNTER(&htim2);
            q.x = s.x;
            q.y = s.y;
            q.z = s.z;
            // never wait here, drop and count
            if (xQueueSend(sample_q, &q, 0) != pdPASS) dropped++;
        }
        vTaskDelayUntil(&wake, SENSOR_PERIOD);
    }
}

static void ring_push(const sample_t *q)
{
    ring[head][0] = q->x;
    ring[head][1] = q->y;
    ring[head][2] = q->z;
    head = (uint16_t)((head + 1u) % MODEL_WIN);
    if (filled < MODEL_WIN) filled++;
}

// oldest first so the model sees time in order
static void ring_unroll(void)
{
    uint16_t n = (uint16_t)(MODEL_WIN - head);
    memcpy(win, ring[head], (size_t)n * MODEL_AXES);
    if (head) memcpy(win[n], ring[0], (size_t)head * MODEL_AXES);
}

static void inference_task(void *arg)
{
    (void)arg;
    sample_t q;
    det_t    st = WAIT_FF;
    uint32_t ff_run = 0;
    uint32_t left = 0;
    uint32_t t_hit = 0;

    Model_Init();

    for (;;) {
        if (xQueueReceive(sample_q, &q, portMAX_DELAY) != pdPASS) continue;
        ring_push(&q);

        int32_t mag2 = (int32_t)q.x * q.x + (int32_t)q.y * q.y + (int32_t)q.z * q.z;

        // cheap rule picks the moment, model makes the call
        switch (st) {
        case WAIT_FF:
            ff_run = (mag2 < FREEFALL_MAG2) ? ff_run + 1u : 0u;
            if (ff_run >= FREEFALL_MIN) {
                ff_run = 0;
                left = IMPACT_WINDOW;
                st = WAIT_IMPACT;
            }
            break;

        case WAIT_IMPACT:
            if (mag2 > IMPACT_MAG2) {
                t_hit = q.t;
                left = POST_IMPACT;
                st = WAIT_POST;
            } else if (mag2 < FREEFALL_MAG2) {
                left = IMPACT_WINDOW;   // still falling
            } else if (--left == 0u) {
                st = WAIT_FF;
            }
            break;

        case WAIT_POST:
            if (--left == 0u) {
                if (filled == MODEL_WIN) {
                    ring_unroll();
                    float score = Model_Run(win);
                    if (score >= MODEL_THRESH) Rtos_OnFall(t_hit, score);
                }
                left = COOLDOWN;
                st = COOL;
            }
            break;

        case COOL:
            if (--left == 0u) st = WAIT_FF;
            break;
        }
    }
}

void Rtos_Init(void)
{
    sample_q = xQueueCreate(QUEUE_LEN, sizeof(sample_t));
    if (sample_q == NULL) Error_Handler();

    // stack is in words, sensor higher prio so it never misses a sample
    if (xTaskCreate(sensor_task, "sensor", 256, NULL,
                    tskIDLE_PRIORITY + 2, NULL) != pdPASS) Error_Handler();
    if (xTaskCreate(inference_task, "infer", 512, NULL,
                    tskIDLE_PRIORITY + 1, NULL) != pdPASS) Error_Handler();
}

uint32_t Rtos_Dropped(void)
{
    return dropped;
}

__attribute__((weak)) void Rtos_OnFall(uint32_t t, float score)
{
    (void)t;
    (void)score;
}

#endif
