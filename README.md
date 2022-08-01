# AVR multi-zone LED controller

The built-in hardware PWM (pulse width modulation) feature of ATmega328p (Arduino Nano/Uno) provides up to 6 analog outputs on select pins, provided through the Arduino API as the `analogWrite` function. However, by careful use of timer interrupts and low-level GPIO control, we can instead use software to generate a PWM signal on _any_ GPIO pin. With 18 free GPIO pins (reserving D0/D1 for serial Tx/Rx), an Arduino Nano/Uno can drive 6 fully independent strings of RGB lights, as demonstrated by this example code.

Distributed under the [MIT license](LICENSE.txt)

## Usage

After building and uploading the program to the Arduino, connect a serial monitor such as the one included with PlatformIO. A '>' should appear as a prompt for input. The following commands are available:

- 

```
>
```

## Dependencies

The following dependencies will be downloaded by PlatformIO at build time:

- [uIO](https://github.com/trevor-makes/uIO)
- [uCLI](https://github.com/trevor-makes/uCLI)
- [uANSI](https://github.com/trevor-makes/uANSI)

## Contributors

[Trevor Makes](mailto:the.trevor.makes@gmail.com)
