# AVR multi-zone LED controller

The AVR timer units in an Arduino Nano allow for up to 6 hardware-controlled PWM outputs on specific pins, which Arduino exposes through the `analogWrite` function. However, by careful use of the timer interrupt and a few extra cycles, we can send PWM signals to any I/O pin using software-control.

With all 18 I/O pins of an Arduino Nano under PWM control (not including D0/D1 used for Tx/Rx), we can drive up to 6 independent strings of RGB lights, as demonstrated in this example code.

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
