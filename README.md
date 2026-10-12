# Digital Cuckoo Clock
A smart digital clock with alarm, environmental monitoring and multimedia features

## Description

My project is a standalone smart digital clock built on an STM32 microcontroller. The device displays the current time on a touchscreen and allows the user to set alarms. When triggered, the alarm can play sound and activate a cuckoo-style mechanical system. It also monitors the air quality, showing this information on the display. It can also play audio from an SD card.

## Motivation

I wanted to make something both useful and creative, so my first instinct was to look around in my room. What caught my eye was the digital clock from my bedside, which gave me even more ideas for things I can add to make this project outstanding. For the creative aspect, after searching the Internet for designs of old clocks, I came acorss a picture of a beautiful cuckoo clock that inspired the final iteration of my idea.

## Architecture 

The system is structured into several main components: the central control unit, the sensing subsystem, the communication module, the multimedia subsystem, and the actuation system. The central control unit, based on the STM32 microcontroller, coordinates all operations and manages communication between subsystems.

The sensing subsystem includes the air quality sensor which provide real-time environmental data to the STM32. This data is used both for display purposes.

The multimedia subsystem includes the display, audio sources, and storage. The TFT touchscreen display communicates with the STM32 via SPI, while the SD card shares the same SPI bus. The FM radio module communicates via I2C. Audio playback is handled by external modules (amplifier), which receive analog signals and output sound to the speakers.

The actuation system includes the servo motor used for the cuckoo mechanism via PWM signals. The STM32 generates control signals using timers.

All components are powered by a 5V USB power supply, with voltage stabilization ensured by capacitors. The STM32 interfaces with peripherals using SPI, ADC, and PWM depending on the device.
