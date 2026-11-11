# Tiny ML Fall Detector - accelerometer data recorder

Firmware and PC tooling for recording labelled fall and daily-activity data
with an STM32 NUCLEO-F411RE and the KA-Nucleo-Multisensor shield.

## Hardware

- NUCLEO-F411RE (STM32F411RET6, ARM Cortex-M4, 100 MHz from the 8 MHz HSE)
- KA-Nucleo-Multisensor shield: LIS35DE accelerometer, 4-digit 7-segment display, two push buttons

| Function | Pins |
| --- | --- |
| SPI1 to LIS35DE (mode 3, 3.125 Mbit/s) | PB3 SCK, PA6 MISO, PA7 MOSI, PA4 CS |
| USART2 to ST-LINK virtual COM port (460800 baud, 8N1) | PA2 TX, PA3 RX |
| Event button / reset button (active low, pull-up) | PB0 / PB5 |
| 7-segment display | PC0-PC7 segments, PC8-PC11 digit commons |

## Firmware

Peripherals are configured in STM32CubeMX (`fall_detector.ioc`) and use the STM32 HAL.

- `Core/Src/lis35de.c` - LIS35DE driver: WHO_AM_I check, 400 Hz output data rate, +-8 g range
- `Core/Src/app.c` - sampling loop, button debounce in the 1 kHz TIM4 interrupt, UART output
- `Core/Src/seg_display.c` - multiplexed 7-segment display showing the recording count
- TIM2 runs free at 1 MHz and timestamps every sample

UART protocol, one ASCII record per line:

```
S,<t_us>,<x>,<y>,<z>   accelerometer sample, raw int8 counts (about 72 mg per count)
E,<id>,<t_us>          event button: recording <id> ended
R,<t_us>               reset button: session ended, counter back to 0
#...                   status text
```

## PC logger

`tools/fall_logger.py` reads the stream, shows a live plot of total acceleration
and saves each recording as one labelled CSV row.

```
pip install pyserial matplotlib
python tools/fall_logger.py --label fall
python tools/fall_logger.py --demo
```

## Signal processing

Recorded data is cleaned offline with a second-order Butterworth low-pass filter
at 35 Hz before further analysis.

## Building

1. Open `fall_detector.ioc` in STM32CubeMX or STM32CubeIDE and generate code. This restores the `Drivers/` folder and the startup file, which are not stored in this repository.
2. Build and flash from STM32CubeIDE.

## Status

Data recording is complete. Fall classification on the device is the next stage.
