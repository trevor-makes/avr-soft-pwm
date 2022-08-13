// Copyright (c) 2022 Trevor Makes

#include "uIO.hpp"
#include "uCLI.hpp"
#include "uPWM.hpp"

#include <Arduino.h>

// This tutorial is a good resource to mention
//https://raw.githubusercontent.com/abcminiuser/avr-tutorials/master/Timers/Output/Timers.pdf

using uCLI::Args;
uCLI::StreamEx serialEx(Serial);
uCLI::CLI<> serialCli(serialEx);

#ifdef __AVR_ATmega328P__
struct PWMTimer {
  // uIO types for Timer2 registers
  uIO_REG(TCCR2A)
  uIO_REG(TCCR2B)
  uIO_REG(TIMSK2)
  uIO_REG(OCR2A)

  // Aliases for Timer2 bitfields
  using RegCOM2A = uIO::RightAlign<RegTCCR2A::Mask<0xC0>>;
  using RegCS2 = RegTCCR2B::Mask<0x07>;
  using RegWGM2 = uIO::BitExtend<RegTCCR2B::Bit<WGM22>, RegTCCR2A::Mask<0x03>>;
  using BitOCIE2A = RegTIMSK2::Bit<OCIE2A>;

  // Config timer to interrupt after variable number of cycles
  // NOTE longest ISR measured was greater than 128 and less than 256 clocks, so use 256 prescaler
  static void config() {
    RegWGM2::write(2); // select CTC mode
    RegCOM2A::write(0); // normal GPIO mode, no hardware PWM on OC2A
    BitOCIE2A::set(); // enable compare match interrupt
    RegCS2::write(6); // set prescaler to 256
  }

  // Interrupt after (delay * prescaler) CPU cycles
  static void set_delay(uint8_t delay) {
    RegOCR2A::write(delay);
  }

  static void disable_isr() {
    BitOCIE2A::clear();
  }
};

// PWM Controller pin mapping
// PortD is most significant (byte 2), PortB is least (byte 0)
using PortD = uIO::PortD::Mask<0xFC>;
using PortC = uIO::PortC::Mask<0x3F>;
using PortB = uIO::PortB::Mask<0x3F>;
using PWMPins = uIO::WordExtend<PortD, PortC, PortB>;

constexpr const uint8_t N_ZONES = 6;
constexpr const uint8_t N_PER_ZONE = 3;
constexpr const uint8_t N_KEYFRAMES = 8;
uPWM::Controller<PWMPins, PWMTimer, N_ZONES, N_PER_ZONE, N_KEYFRAMES> pwm;

// Hook PWM routine into timer 2 compare interrupt
ISR(TIMER2_COMPA_vect) {
  pwm.isr();
}

// Timer0 overflow used by Arduino for micros/millis/etc
struct MicrosTimer {
  uIO_REG(TIMSK0)

  static void enable_isr() {
    RegTIMSK0::Bit<TOIE0>::set();
  }

  static void disable_isr() {
    RegTIMSK0::Bit<TOIE0>::clear();
  }
};
#else
#error Need to provide configuration for current platform. See __AVR_ATmega328P__ configuration above.
#endif

void set_default(Args);
void set_rainbow(Args);

void setup() {
  PWMPins::config_output();

  // Configure mapping from zone/channel to bit within port register
  //     7  6  5  4  3  2  1  0
  // B [ -  - g3 r3 b3 g4 r4 b4]
  // C [ -  - g0 r0 b0 g1 r1 b1]
  // D [g2 r2 b2 g5 r5 b5  -  -]
  constexpr auto B = 0, C = 8, D = 16;
  pwm.config_pins(0, C + 4, C + 5, C + 3); // r0, g0, b0
  pwm.config_pins(1, C + 1, C + 2, C + 0); // r1, g1, b1
  pwm.config_pins(2, D + 6, D + 7, D + 5); // r2, g2, b2
  pwm.config_pins(3, B + 4, B + 5, B + 3); // r3, g3, b3
  pwm.config_pins(4, B + 1, B + 2, B + 0); // r4, g4, b4
  pwm.config_pins(5, D + 3, D + 4, D + 2); // r5, g5, b5

  set_default({});

  PWMTimer::config();

  // OPTIONAL Disable Arduino micros interrupt (will break micros/millis/delay/etc!)
  // Software PWM will work without disabling this, but it causes jitter observable with a scope
  // I couldn't tell the difference with LEDs, but it might be noticible with motors or audio
  // Feel free to remove this if micros is needed
  MicrosTimer::disable_isr();

  // Establish serial connection with computer
  Serial.begin(9600);
  while (!Serial) {}
}

void do_list(Args);
void do_clear(Args);
void config_channel(Args);
void set_keyframe(Args);
void set_period(Args);
void measure_isr(Args);

constexpr const char* KEYFRAME_CMD = "keyframe";
constexpr const char* PERIOD_CMD = "period";

void loop() {
  static const uCLI::Command commands[] = {
    { "config", config_channel },
    { KEYFRAME_CMD, set_keyframe },
    { PERIOD_CMD, set_period },
    { "clear", do_clear },
    { "default", set_default },
    { "rainbow", set_rainbow },
    { "list", do_list },
    { "measure", measure_isr },
  };

  // Update PWM animation while waiting for CLI input
  serialCli.run_once(commands, []() { pwm.update(); });
}

void set_default(Args) {
  pwm.clear_all();
  constexpr uint8_t TIME = 0;
  pwm.set_keyframe(0, TIME, 255, 0, 191);
  pwm.set_keyframe(1, TIME, 191, 0, 63);
  pwm.set_keyframe(2, TIME, 191, 31, 63);
  pwm.set_keyframe(3, TIME, 191, 63, 63);
  pwm.set_keyframe(4, TIME, 255, 63, 15);
  pwm.set_keyframe(5, TIME, 255, 31, 0);
}

void set_rainbow(Args) {
  pwm.clear_all();
  pwm.set_period(1500);
  for (uint8_t i = 0; i < 6; ++i) {
    pwm.set_keyframe(i, 0 + i * 100, 255, 0, 0);
    pwm.set_keyframe(i, 500 + i * 100, 0, 255, 0);
    pwm.set_keyframe(i, 1000 + i * 100, 0, 0, 255);
  }
}

void do_clear(Args args) {
  if (args.has_next()) {
    uint8_t zone = atoi(args.next());
    pwm.clear_zone(zone);
  } else {
    pwm.clear_all();
  }
}

void config_channel(Args args) {
  uint8_t zone = atoi(args.next());
  uint8_t red = atoi(args.next());
  uint8_t green = atoi(args.next());
  uint8_t blue = atoi(args.next());
  pwm.config_pins(zone, red, green, blue);
}

void set_keyframe(Args args) {
  uint8_t zone = atoi(args.next());
  uint16_t time = atoi(args.next());
  uint8_t red = atoi(args.next());
  uint8_t green = atoi(args.next());
  uint8_t blue = atoi(args.next());
  pwm.set_keyframe(zone, time, red, green, blue);
}

void set_period(Args args) {
  pwm.set_period(atoi(args.next()));
}

template <typename T>
void print_list(const T value) {
  serialEx.println(value);
}

template <typename T, typename... Args>
void print_list(const T value, const Args... args) {
  serialEx.print(value);
  serialEx.print(' ');
  print_list(args...);
}

void do_list(Args args) {
  print_list(PERIOD_CMD, pwm.get_period());

  for (uint8_t zone = 0; zone < N_ZONES; ++zone) {
    uint16_t time;
    uint8_t red, green, blue;

    // Loop over each keyframe
    uint8_t i = 0;
    while (pwm.get_keyframe(zone, i++, time, red, green, blue)) {
      print_list(KEYFRAME_CMD, zone, time, red, green, blue);
    }
  }
}

void measure_isr(Args args) {
  serialEx.println("Connect scope to pin D0 (Rx) and single trigger on pulse width > 1 us to measure ISR");

  // Disable Rx on pin D0 so we can use it as a digital I/O
  Serial.end();
  using MeasurePin = uIO::PinD0;
  MeasurePin::config_output();

  // Multiple ISRs can fire back-to-back and appear as one longer ISR
  // Use these options to isolate one ISR at a time
  if (args.has_next()) {
    auto next = args.next();
    if (strcmp(next, "both") == 0) {
      MicrosTimer::enable_isr();
    } else if (strcmp(next, "micros") == 0) {
      MicrosTimer::enable_isr();
      // Only want micros, so disable PWM
      PWMTimer::disable_isr();
    } else if (strcmp(next, "pwm") == 0) {
      // Only want PWM, so disable micros
      MicrosTimer::disable_isr();
    }
  }

  // Toggle pin in an infinite loop to generate a 2 MHz square wave (16 MHz / 8 CPU cycles)
  // When an ISR runs it will interrupt the loop and the wave will be stuck high or low like so:
  // -| 8 |- CPU cycles (0.5 us)
  //  |   |   |- ISR length -|     |- ISR length -|
  // _/-\_/-\_:______________/-\_/-:--------------\_
  // Measure the pulse width minus 1/2 period (4 CPU cycles, 0.25 us) to compute the ISR duration
  // Reset the board to exit the loop and resume normal function
  for (;;) { // 8 CPU cycles per loop
    MeasurePin::set(); // SBI, 2 cycles
    __asm__ __volatile__ ("nop\n nop\n"); // 2 cycles (1 per NOP)
    MeasurePin::clear(); // CBI, 2 cycles
  } // RJMP, 2 cycles
}
