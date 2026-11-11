#include "seg_display.h"
#include "main.h"

/* ---------------------------------------------------------------------------
 * POLARITY: the shield documentation does not say whether DISP1 (7SD4-056) is
 * common-cathode or common-anode. 1 = common cathode (segment HIGH = lit,
 * digit common LOW = selected). If the display shows nonsense or is very dim
 * and every digit looks like an inverted "8", flip this to 0.
 * ------------------------------------------------------------------------- */
#define DISP_COMMON_CATHODE 1

#if DISP_COMMON_CATHODE
#define SEG_ON_LEVEL 1u
#define COM_ON_LEVEL 0u
#else
#define SEG_ON_LEVEL 0u
#define COM_ON_LEVEL 1u
#endif

/* Port C bit number for each segment bit: A B C D E F G DP (from schematic) */
static const uint8_t SEG_PIN[8] = { 6, 4, 3, 1, 2, 5, 0, 7 };

/* Port C bit number for each digit common, left digit first.
 * If the number appears mirrored ("100" instead of "001"), reverse this. */
static const uint8_t COM_PIN[4] = { 11, 10, 9, 8 };

/* Segment patterns, bit0 = A ... bit6 = G, bit7 = DP. Index 0-9 = digits. */
enum { G_BLANK = 10, G_DASH = 11, G_E = 12, G_r = 13 };
static const uint8_t FONT[14] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F,  /* 0-9  */
    0x00,                                                        /* blank */
    0x40,                                                        /* -     */
    0x79,                                                        /* E     */
    0x50                                                         /* r     */
};

/* What each digit should show; written by main loop, read by the ISR. */
static volatile uint8_t frame[4];

void Disp_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gi = {0};
    gi.Pin   = 0x0FFF;                       /* PC0 .. PC11 */
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gi);

    Disp_ShowDashes();
}

void Disp_Tick(void)
{
    static uint8_t d = 0;                    /* digit being driven this tick */
    uint32_t out = 0;
    uint8_t seg = frame[d];

    for (uint8_t i = 0; i < 8; i++) {
        uint32_t lit   = (seg >> i) & 1u;
        uint32_t level = lit ? SEG_ON_LEVEL : (SEG_ON_LEVEL ^ 1u);
        out |= level << SEG_PIN[i];
    }
    for (uint8_t k = 0; k < 4; k++) {
        uint32_t level = (k == d) ? COM_ON_LEVEL : (COM_ON_LEVEL ^ 1u);
        out |= level << COM_PIN[k];
    }

    /* One atomic write: low half sets pins, high half clears them. */
    GPIOC->BSRR = (out & 0x0FFFu) | ((~out & 0x0FFFu) << 16);

    d = (uint8_t)((d + 1u) & 3u);
}

void Disp_SetNumber(uint16_t n)
{
    if (n > 9999u) n = 9999u;
    frame[0] = (n >= 1000u) ? FONT[(n / 1000u) % 10u] : FONT[G_BLANK];
    frame[1] = FONT[(n / 100u) % 10u];       /* zero-padded: shows " 001" */
    frame[2] = FONT[(n / 10u) % 10u];
    frame[3] = FONT[n % 10u];
}

void Disp_ShowErr(void)
{
    frame[0] = FONT[G_BLANK];
    frame[1] = FONT[G_E];
    frame[2] = FONT[G_r];
    frame[3] = FONT[G_r];
}

void Disp_ShowDashes(void)
{
    for (uint8_t i = 0; i < 4; i++) frame[i] = FONT[G_DASH];
}
