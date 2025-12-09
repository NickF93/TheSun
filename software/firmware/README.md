# TheSun Firmware (ATtiny1616)

Firmware for the “TheSun” lamp controller running on ATtiny1616 with an SSD1306 OLED and AHT20 temperature/humidity sensor. PWM drives the LED on PB5 (Arduino pin 4) using TCA0 WO2.

## Overview

- Drives an LED via hardware PWM on PB5 with duty mapped from user “percentage.”
- Reads an AHT20 over I2C for temperature/humidity.
- Optionally reads NTC thermistors (enable `NTC1`/`NTC2` defines).
- Shows status on a 128x64 SSD1306 display.
- Uses four buttons on PC0–PC3 for fine/coarse percentage up/down.

## Hardware

- MCU: ATtiny1616 @ 16 MHz (megaTinyCore)
- Display: SSD1306 128x64 via I2C (addr 0x3C)
- Sensor: AHT20 via I2C
- LED/PWM: PB5 (Arduino pin 4), TCA0 WO2
- Buttons: PC0–PC3 (Arduino pins 10–13), INPUT_PULLUP
- Optional NTCs: define `NTC1`/`NTC2` to enable analog reads

## Build & Flash

- PlatformIO env: `thesun_attiny1616`
- Critical flag: `-DTCA_PORTMUX=PORTMUX_TCA02_bm` (routes TCA0 WO2 to PB5)
- Upload: serialUPDI (per `platformio.ini`)
- Clock: internal 16 MHz; see `platformio.ini` for fuses/BOD/EEPROM keep

## Behavior & Timing

- PWM: ~250 Hz (TCA0 single-slope, PER=1000, prescaler=64), single channel WO2.
- Percentage mapping: input 0–100 → mapped 5–50; PWM duty = `(mapped * 255) / 100`
  - 0% → mapped ~5 → duty ~4–5% (with PER=1000)
  - 100% → mapped ~50 → duty ~50%
- Scheduling: buttons every 50 ms; temperature/humidity every 5 s; OLED every 5 s or when data changes.
- TCA0 ownership: `takeOverTCA0()` removes TCA0 from core control. Millis must not use TCA0; prefer TCB/RTC timebase in core settings.

## Configuration Notes

- If the PWM pin changes, update `LED_PWM_PIN`, the portmux flag, and enable the matching TCA compare channel.
- NTC averaging uses 100 samples per read; lower `NTC_BUFFER` to reduce CPU load.
- AHT20 reads: last good values are kept on I2C failure.

## Troubleshooting

- No PWM on PB5: verify `-DTCA_PORTMUX=PORTMUX_TCA02_bm` is present and TCA0 isn’t reconfigured elsewhere.
- millis/delay broken: ensure core is set to use TCB/RTC for millis, not TCA0.
- Display empty: confirm I2C pull-ups and address 0x3C.
