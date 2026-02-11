# Pico RP2040 → BNO055 Wiring Diagram

## Overview

A Raspberry Pi Pico (RP2040) reads the BNO055 9-DOF sensor via I2C and
streams fused orientation + raw sensor data over USB serial to the Pi 5.

```
  ┌─────────────────────────────────────────────────────────┐
  │                   Raspberry Pi 5                        │
  │                                                         │
  │   USB-A port ◄──── USB cable ────► Pico USB (micro-B)  │
  │                                                         │
  └─────────────────────────────────────────────────────────┘


         Raspberry Pi Pico (RP2040)
         ┌──────────────────────────────────────┐
         │                USB                   │
         │              ┌─────┐                 │
  GP0  1 │█             │     │              █ 40  VBUS
  GP1  2 │█             └─────┘              █ 39  VSYS
  GND  3 │█                                  █ 38  GND
  GP2  4 │█                                  █ 37  3V3_EN
  GP3  5 │█                                  █ 36  3V3(OUT) ──────┐
  GP4  6 │█  ◄── I2C0 SDA ──────────────┐   █ 35  ADC_VREF      │
  GP5  7 │█  ◄── I2C0 SCL ──────────┐   │   █ 34  GP28          │
  GND  8 │█ ─────────────────────┐   │   │   █ 33  GND           │
  GP6  9 │█                      │   │   │   █ 32  GP27          │
  GP7 10 │█                      │   │   │   █ 31  GP26          │
  GP8 11 │█                      │   │   │   █ 30  RUN           │
  GP9 12 │█                      │   │   │   █ 29  GP22          │
  GND 13 │█                      │   │   │   █ 28  GND           │
 GP10 14 │█                      │   │   │   █ 27  GP21          │
 GP11 15 │█                      │   │   │   █ 26  GP20          │
 GP12 16 │█                      │   │   │   █ 25  GP19          │
 GP13 17 │█  ◄── LED (optional)  │   │   │   █ 24  GP18          │
  GND 18 │█                      │   │   │   █ 23  GND           │
 GP14 19 │█                      │   │   │   █ 22  GP17          │
 GP15 20 │█                      │   │   │   █ 21  GP16          │
         └──────────────────────────────────────┘
                                 │   │   │                        │
                                 │   │   │                        │
         GY-BNO055 Module        │   │   │                        │
         ┌────────────────┐      │   │   │                        │
         │     BNO055     │      │   │   │                        │
         │                │      │   │   │                        │
         │  VIN ──────────│──────│───│───│────────────────────────┘
         │  GND ──────────│──────┘   │   │
         │  SDA ──────────│──────────│───┘
         │  SCL ──────────│──────────┘
         │  ADR ──────────│── (leave floating or tie LOW → 0x28)
         │  RST ──────────│── (leave floating, has internal pull-up)
         │  INT ──────────│── (unused)
         │  PS0 ──────────│── (leave floating or tie LOW → I2C mode)
         │  PS1 ──────────│── (leave floating or tie LOW → I2C mode)
         │                │
         └────────────────┘
```

## Pin Connections Summary

| BNO055 Pin | Pico Pin           | Pico Label   | Notes                         |
|------------|--------------------|--------------|-------------------------------|
| **VIN**    | Pin 36             | 3V3(OUT)     | 3.3 V regulated output        |
| **GND**    | Pin 8 (or any GND) | GND          |                               |
| **SDA**    | Pin 6              | GP4          | I2C0 SDA, 3.3 V logic        |
| **SCL**    | Pin 7              | GP5          | I2C0 SCL, 3.3 V logic        |
| **ADR**    | —                  | —            | Float/LOW = address **0x28**  |
| **RST**    | —                  | —            | Float (internal pull-up)      |
| **INT**    | —                  | —            | Not used                      |
| **PS0**    | —                  | —            | Float/LOW → I2C mode          |
| **PS1**    | —                  | —            | Float/LOW → I2C mode          |

## Optional: Status LED

Connect an LED + 330 Ω resistor from **GP13** (pin 17) to **GND** for
a visual heartbeat / calibration indicator.

## I2C Pull-ups

The GY-BNO055 breakout board already includes 10 kΩ pull-ups on SDA/SCL.
**No external pull-ups needed.**

## BNO055 Address

| ADR Pin | I2C Address |
|---------|-------------|
| LOW/Float | **0x28**  |
| HIGH      | 0x29      |

## USB Connection

Connect the Pico's micro-USB port to any free USB port on the Pi 5.
It will appear as `/dev/ttyACM0` or `/dev/ttyACM1` (whichever is not
already taken by the Arduino Nano RP2040).

### Identifying which ACM port is which

```bash
# List USB serial devices with vendor info:
ls -la /dev/serial/by-id/

# The Pico will show as "Raspberry_Pi_Pico"
# The Arduino Nano will show as "Arduino"
```

## Power Budget

| Device      | Current Draw |
|-------------|-------------|
| BNO055      | ~12 mA (all sensors active, fusion running) |
| Pico RP2040 | ~25 mA (idle USB) |
| **Total**   | ~37 mA from Pi 5 USB — well within limits |
