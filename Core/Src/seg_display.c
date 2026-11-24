#include "seg_display.h"
#include "main.h"

// not in the shield docs, set 0 if its common anode
#define DISP_COMMON_CATHODE 1

#if DISP_COMMON_CATHODE
#define SEG_ON_LEVEL 1u
#define COM_ON_LEVEL 0u
#else
#define SEG_ON_LEVEL 0u
#define COM_ON_LEVEL 1u
#endif

static const uint8_t SEG_PIN[8] = { 6, 4, 3, 1, 2, 5, 0, 7 };

static const uint8_t COM_PIN[4] = { 11, 10, 9, 8 };

enum { G_BLANK = 10, G_DASH = 11, G_E = 12, G_r = 13 };
static const uint8_t FONT[14] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F,
    0x00,
    0x40,
    0x79,
    0x50
};

static volatile uint8_t frame[4];

void Disp_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gi = {0};
    gi.Pin   = 0x0FFF;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gi);

    Disp_ShowDashes();
}

void Disp_Tick(void)
{
    static uint8_t d = 0;
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

    // set and clear in one write
    GPIOC->BSRR = (out & 0x0FFFu) | ((~out & 0x0FFFu) << 16);

    d = (uint8_t)((d + 1u) & 3u);
}

void Disp_SetNumber(uint16_t n)
{
    if (n > 9999u) n = 9999u;
    frame[0] = (n >= 1000u) ? FONT[(n / 1000u) % 10u] : FONT[G_BLANK];
    frame[1] = FONT[(n / 100u) % 10u];
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

void Disp_ShowWord(const char *s)
{
    for (uint8_t i = 0; i < 4; i++) {
        char c = s[i];
        uint8_t seg = 0x00;
        if (c >= '0' && c <= '9') seg = FONT[c - '0'];
        else switch (c) {
            case '-': seg = 0x40; break;
            case 'E': seg = 0x79; break;
            case 'r': seg = 0x50; break;
            case 'F': seg = 0x71; break;
            case 'U': seg = 0x3E; break;
            case 'L': seg = 0x38; break;
            case 'S': seg = 0x6D; break;
            case 'A': seg = 0x77; break;
            case 'C': seg = 0x39; break;
            case 'd': seg = 0x5E; break;
            case 'n': seg = 0x54; break;
            case 'o': seg = 0x5C; break;
            case 't': seg = 0x78; break;
            case 'P': seg = 0x73; break;
            case 'b': seg = 0x7C; break;
            case 'H': seg = 0x76; break;
            default:  seg = 0x00; break;
        }
        frame[i] = seg;
    }
}
