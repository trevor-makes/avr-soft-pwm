# ATmega328p 6-strip RGB LED controller

The built-in hardware PWM ([pulse-width modulation](https://en.wikipedia.org/wiki/Pulse-width_modulation)) feature of AVR microcontrollers allows certain GPIO pins to be pulsed by a hardware timer, approximating an analog signal proportional to how long a pin is in a high or low state. The duty cycle is set by software (the `analogWrite` function from the Arduino API), and then the timer will continue to pulse the pin independent of the CPU.

The ATmega328p (Arduino Nano/Uno) has 3 timers that can drive up to 6 PWM outputs, suitable for 2 strips of RGB LEDs. But what about the other 12 GPIO pins?

By careful use of timer interrupts and low-level GPIO control, we can instead use software to generate a PWM signal on _any_ GPIO pin (or all at once). With 18 free GPIO pins (reserving D0/D1 for serial Tx/Rx), an Arduino Nano/Uno can drive 6 fully independent strings of RGB lights.

Distributed under the [MIT license](LICENSE.txt)

## Building

Use the [PlatformIO](https://platformio.org/) plugin for [VSCode](https://code.visualstudio.com/).

Open the project folder with VSCode, select the environment for your board (`uno`, `nano`, `oldnano`), and click `Upload`.

## Using the example program

(TODO circuit diagram)

After building and uploading the program to the Arduino, connect a serial monitor such as the one included with PlatformIO. A '>' should appear as a prompt for input. The following commands are available:

```
>keyframe zone time red green blue
```
Add an RGB keyframe to `zone` at the specified `time` (250 ticks = 1 second). A single keyframe will display a fixed color, while multiple keyframes will interpolate over time to display a gradient.

```
>period time
```
Set the animation to loop over `time` ticks (250 ticks = 1 second).

```
>clear (zone)
```
Clear all keyframes from the given `zone`, or clear all zones if none given.

```
>list
```
Prints `period` and `keyframe` commands currently in use. These can be copy-pasted and resent to recreate the current display.

```
>measure (pwm|micros|both)
```
Freezes the animation and command input while generating a square wave on the `Rx` pin. Timer interrupts will continue to run and can be observed with an oscilloscope or logic analyzer where the square wave is jammed high or low. Refer to the source code for instructions on measuring ISR duration.

(TODO scope screenshot)

## Using the uPWM library

(TODO)

## Dependencies

The following dependencies will be downloaded by PlatformIO at build time:

- [uIO](https://github.com/trevor-makes/uIO)
- [uCLI](https://github.com/trevor-makes/uCLI)
- [uANSI](https://github.com/trevor-makes/uANSI)

## Contributors

[Trevor Makes](mailto:the.trevor.makes@gmail.com)
