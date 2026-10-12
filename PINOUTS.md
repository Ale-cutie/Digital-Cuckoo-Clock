# Project Pinouts

Pin reference for the STM32U545RE project wiring used by the display, touch,
SD card, servo, and air quality sensor examples.

## LCD Display - SPI1

| Board/Header | MCU Pin | Signal |
| --- | --- | --- |
| D8 | PC7 | LCD RESET |
| D9 | PC6 | LCD DC |
| D10 | PC9 | LCD CS |
| D11 | PA7 | SPI1 MOSI |
| D12 | PA6 | SPI1 MISO |
| D13 | PA5 | SPI1 SCK |
| 3V3 | 3.3 V | VCC, LED |
| GND | GND | Ground |

## Touch Controller - SPI1

| Board/Header | MCU Pin | Signal |
| --- | --- | --- |
| D14 | PB7 | Touch CS |
| D15 | PB6 | Touch IRQ |
| D11 | PA7 | SPI1 MOSI |
| D12 | PA6 | SPI1 MISO |
| D13 | PA5 | SPI1 SCK |
| 3V3 | 3.3 V | VCC |
| GND | GND | Ground |

The touch controller shares SPI1 with the LCD, using its own chip-select pin.

## SD Card - SPI1 (shared with LCD and Touch)

| Board/Header | MCU Pin | Signal |
| --- | --- | --- |
| D13 | PA5 | SPI1 SCK |
| D11 | PA7 | SPI1 MOSI |
| D12 | PA6 | SPI1 MISO |
| CN10-22 | PB2 | SD CS |
| 3V3 or module VCC | Power | SD VCC |
| GND | GND | Ground |

Notes:

- CN10-16 is NC on this board (UM3062); SD CS uses CN10-22 / PB2 instead.
- PA6 / SPI1 MISO has internal pull-up enabled in code.
- If SD reads fail, add an external 10k pull-up from PA6 to 3V3.
- Check the SD breakout power requirement. Some modules accept 5 V input,
  while bare microSD sockets usually require 3.3 V.

## Servo

| MCU Pin | Timer Channel | Signal |
| --- | --- | --- |
| PA8 | TIM1_CH1 | Servo PWM signal |

Power wiring:

| Servo Wire | Connection |
| --- | --- |
| Signal | PA8 |
| VCC | 5V |
| GND | Common GND with STM32 board |

Notes:

- PWM frequency is 50 Hz.
- Current configured sweep is 0 to 90 degrees.

## PAM8403 Audio Amplifier

| PAM8403 pin | MCU/Board pin | Signal |
| --- | --- | --- |
| +5V | 5V | Amplifier power |
| GND | GND | Common ground |
| L / LIN | PA4 through 1k resistor | Mono DAC audio input |
| R / RIN | PA4 through 1k resistor | Same mono DAC audio input |
| SW | PA1 | Amplifier enable / mute control |
| L+, L- | Left speaker | Floating speaker output, do not connect to GND |
| R+, R- | Right speaker | Floating speaker output, do not connect to GND |

Notes:

- The code drives PA1 low except while the Music page is open and Play is active.
- If your PAM8403 module's SW pin behaves inverted, swap the `set_high()` / `set_low()` calls for `amp_sw` in `src/main.rs`.
- Keep speaker outputs isolated from board ground; PAM8403 outputs are bridge-tied.

## Air Quality Sensor - MQ-135

| MCU Pin | Peripheral | Signal |
| --- | --- | --- |
| PA0 | ADC1 CH0 | MQ-135 analog output |

Power wiring:

| Sensor Pin | Connection |
| --- | --- |
| AOUT | PA0 |
| VCC | Sensor supply voltage |
| GND | Common GND with STM32 board |

Notes:

- The code assumes a 3.3 V ADC reference (`VDDA_MV = 3300`).
- Make sure the sensor analog output does not exceed 3.3 V on PA0.
- The MQ-135 example logs a warning if the ADC reading is near 0 or saturated.
