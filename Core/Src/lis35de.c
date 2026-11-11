#include "lis35de.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

/* Chip-select: PA4. If you labelled the pin LIS_CS in CubeMX these come from
 * main.h; otherwise the fallback below kicks in. */
#ifndef LIS_CS_Pin
#define LIS_CS_Pin        GPIO_PIN_4
#define LIS_CS_GPIO_Port  GPIOA
#endif

#define CS_LOW()   HAL_GPIO_WritePin(LIS_CS_GPIO_Port, LIS_CS_Pin, GPIO_PIN_RESET)
#define CS_HIGH()  HAL_GPIO_WritePin(LIS_CS_GPIO_Port, LIS_CS_Pin, GPIO_PIN_SET)

/* ---- LIS35DE registers (datasheet section 7) ---- */
#define REG_WHO_AM_I   0x0F
#define REG_CTRL1      0x20
#define REG_STATUS     0x27
#define REG_OUT_X      0x29
#define REG_OUT_Y      0x2B
#define REG_OUT_Z      0x2D

/* CTRL_REG1 bits: DR PD FS STP STM Zen Yen Xen */
#define CTRL1_DR_400HZ 0x80   /* 0 = 100 Hz, 1 = 400 Hz            */
#define CTRL1_PD_ON    0x40   /* 0 = power-down, 1 = active         */
#define CTRL1_FS_8G    0x20   /* 0 = +-2 g range, 1 = +-8 g range   */
#define CTRL1_ZEN      0x04
#define CTRL1_YEN      0x02
#define CTRL1_XEN      0x01

#define STATUS_ZYXDA   0x08   /* new data on all three axes */

#define WHO_AM_I_VALUE 0x3B

/* SPI frame: first byte = [RW][MS][A5..A0]. RW=1 -> read. MS=0 -> no
 * address auto-increment (we read each axis register on its own). */
static uint8_t read_reg(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(0x80u | reg), 0x00 };
    uint8_t rx[2] = { 0, 0 };

    CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 10);
    CS_HIGH();
    return rx[1];
}

static void write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x3Fu), value };

    CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 2, 10);
    CS_HIGH();
}

bool Lis_Init(uint8_t *who)
{
    CS_HIGH();
    HAL_Delay(10);                          /* power-up / boot time */

    *who = read_reg(REG_WHO_AM_I);
    if (*who != WHO_AM_I_VALUE) {
        return false;
    }

    write_reg(REG_CTRL1, CTRL1_DR_400HZ | CTRL1_PD_ON | CTRL1_FS_8G |
                         CTRL1_ZEN | CTRL1_YEN | CTRL1_XEN);

    /* Read it back so a wrong SPI mode shows up as an init failure. */
    return read_reg(REG_CTRL1) ==
           (CTRL1_DR_400HZ | CTRL1_PD_ON | CTRL1_FS_8G |
            CTRL1_ZEN | CTRL1_YEN | CTRL1_XEN);
}

bool Lis_DataReady(void)
{
    return (read_reg(REG_STATUS) & STATUS_ZYXDA) != 0;
}

void Lis_ReadXYZ(lis_sample_t *s)
{
    s->x = (int8_t)read_reg(REG_OUT_X);
    s->y = (int8_t)read_reg(REG_OUT_Y);
    s->z = (int8_t)read_reg(REG_OUT_Z);
}
