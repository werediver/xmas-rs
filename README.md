# xmas-rs

A simple single-channel PWM Christmas lights controller based on a Raspberry Pi Pico (RP2040) board.

![Assembly](images/01%20Assembly.jpeg)

## Hardware

The original configuration is as follows:

- the LED assembly is powered from three AA batteries connected in series through a 2.6Ω resistor
- the load current is about 400 mA
- the voltage drop on the LED assembly is 3.06 V
- the batteries get depleted in a few hours, the brightness drops over time

The modified configuration is as follows:

- the LEDs are powered from the switching mode voltage converter on the Pico board (1.8–5.5 V in, 3.3 V out)
- a P-channel MOSFET TSM2301A is used to commutate the load
- electrolytic capacitors 2x 22 uF are added after the converter to back the pulsed load
- slow "breathing" effect is implemented through PWM modulation
- the LED brightness stays the same till the battery is completely depleted

The Pico board is mounted on the exterior of the original battery case. The MOSFET and capacitors are mounted inside the case.

## Perceived brightness

Note that the relation between PWM duty cycle and the perceived brightness is non-linear. Human eyes distinguish minor brightness changes in dim light and only much larger changes in bright light. Notice how in [this recording](https://odysee.com/@werediver:d/xmas-lights:9) the linear change in PWM duty cycle (active low) is translated into the (perceived) brightness level.

The correction used in this project is as follows:

$$k(b) = {10^b - 1 \over 9}$$

where

- $b \in [0, 1]$ is the desired perceived brightness as a fraction of the maximum brightness
- $k(b) \in [0, 1]$ is the duty cycle

This comes from the (arguably more straightforward) inverse relation:

$$B \propto log(L)$$

where

- $L$ is the (objective) luminous power
- $B$ is the perceived brightness level

I didn't spend enough time to find a research paper to back this. My source is an [SE answer](https://electronics.stackexchange.com/a/550434).
