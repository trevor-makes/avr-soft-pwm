// Copyright (c) 2022 Trevor Makes

#include "uIO.hpp"
#include "uCLI.hpp"

#include "pwm.hpp"

#include <Arduino.h>
//#include <avr/interrupt.h>

// This tutorial is a good resource to mention
//https://raw.githubusercontent.com/abcminiuser/avr-tutorials/master/Timers/Output/Timers.pdf

// TODO move Timer/TimerChannel to uIO?
template <typename TCNT, typename CS, typename WGM, typename TOIE>
struct Timer {
  using counter = TCNT;
  using prescaler = CS;
  using waveform = WGM;
  using interrupt = TOIE;
};

template <typename OCR, typename COM, typename OCIE>
struct TimerCompare {
  using value = OCR;
  using output_mode = COM;
  using interrupt = OCIE;
};

using uCLI::StreamEx;
using uCLI::Args;
StreamEx serialEx(Serial);
uCLI::CLI<> serialCli(serialEx);

#ifdef __AVR_ATmega328P__
// uIO types for Timer2 registers
uIO_REG(TCCR2A)
uIO_REG(TCCR2B)
uIO_REG(TCNT2)
uIO_REG(TIMSK2)
uIO_REG(OCR2A)

// Aliases for Timer2 registers
using RegCOM2A = uIO::RightAlign<RegTCCR2A::Mask<0xC0>>;
using RegCS2 = RegTCCR2B::Mask<0x07>;
using RegWGM2 = uIO::BitExtend<RegTCCR2B::Mask<0x08>, RegTCCR2A::Mask<0x03>>;
using BitOCIE2A = RegTIMSK2::Bit<OCIE2A>;
using BitTOIE2 = RegTIMSK2::Bit<TOIE2>;

// Timer2 type wrapper
constexpr const uint8_t WAVEFORM_CTC = 2;
constexpr const uint8_t OUTPUT_MODE_OFF = 0;
constexpr const uint8_t PRESCALE_64 = 4;
using Timer2 = Timer<RegTCNT2, RegCS2, RegWGM2, BitTOIE2>;
using Timer2A = TimerCompare<RegOCR2A, RegCOM2A, BitOCIE2A>;

// SoftwarePWM pin mapping
constexpr const uint8_t PWM_ZONES = 6;
constexpr const uint8_t PWM_CHANNELS = 3;
using PWMPins = uIO::WordExtend<
  uIO::WordExtend<uIO::PortB::Mask<0x3F>, uIO::PortC::Mask<0x3F>>,
  uIO::WordExtend<uIO::PortD::Mask<0xFC>>>;
using PWM = SoftwarePWM<PWMPins, Timer2A::value, PWM_ZONES, PWM_CHANNELS>;

// Hook PWM routine into timer 2 compare interrupt
ISR(TIMER2_COMPA_vect) {
  PWM::isr();
}
#else
#error Need to provide configuration for current platform. See __AVR_ATmega328P__ configuration above.
#endif

void setup() {
  // TODO should SoftwarePWM take care of pin config?
  PWMPins::config_output();

  // Configure mapping from zone/channel to bit within port register
  PWM::config(0,  8 + 4,  8 + 5,  8 + 3); // C4, C5, C3
  PWM::config(1,  8 + 1,  8 + 2,  8 + 0); // C1, C2, C0
  PWM::config(2, 16 + 6, 16 + 7, 16 + 5); // D6, D7, D5
  PWM::config(3,  0 + 4,  0 + 5,  0 + 3); // B4, B5, B3
  PWM::config(4,  0 + 1,  0 + 2,  0 + 0); // B1, B2, B0
  PWM::config(5, 16 + 3, 16 + 4, 16 + 2); // D3, D4, D2

  // Set RGB value (duty cycle) to default gradient
  PWM::set(0, 255, 0, 191);
  PWM::set(1, 191, 0, 63);
  PWM::set(2, 191, 31, 63);
  PWM::set(3, 191, 63, 63);
  PWM::set(4, 255, 63, 15);
  PWM::set(5, 255, 31, 0);
  PWM::update();

#ifdef __AVR_ATmega328P__
  // Configure hardware timer
  Timer2::waveform::write(WAVEFORM_CTC); // enable CTC mode
  Timer2A::output_mode::write(OUTPUT_MODE_OFF); // disconnect OC2A output
  Timer2A::interrupt::set(); // enable compare interrupt
  Timer2::prescaler::write(PRESCALE_64); // divide by 64
#else
#error TODO make this generic for platforms with different timer configs
#endif

  // Establish serial connection with computer
  Serial.begin(9600);
  while (!Serial) {}
}

void debug_pwm(Args);
void do_list(Args);
void do_pulse(Args);
void set_rgb(Args);
void set_scale(Args);

uCLI::IdleFn idle_fn = nullptr;

void loop() {
  static const uCLI::Command commands[] = {
    { "pulse", do_pulse },
    { "set", set_rgb },
    { "scale", set_scale },
    { "list", do_list },
    { "debug", debug_pwm },
  };

  serialCli.run_once(commands, idle_fn);
}

void debug_pwm(Args) {
  PWM::for_each_event<>([](PWMEvent<PWM::TYPE>* event) {
    // Print the value of each output bit this frame
    // TODO fix the leading zeros so bits line up vertically, like format_hex in uMon
    serialEx.print(event->bits, BIN);
    serialEx.print(" for ");
    // Print duty cycle of this frame (ticks * 100 / 255)
    // NOTE [/ 256] is within decimal precision of and much faster than [/ 255] (should compile to [>> 8])
    uint16_t ticks = event->delay + 1;
    serialEx.print((ticks * 100 + 127) / 256);
    serialEx.println("%");
  });
}

void do_list(Args args) {
  PWM::for_each_channel<>([](PWMChannel<PWM::TYPE>* info, uint8_t zone, uint8_t channel) {
    if (channel == 0) {
      if (zone != 0) {
        serialEx.println();
      }
      serialEx.print(zone);
      serialEx.print(": ");
    } else {
      serialEx.print(", ");
    }
    serialEx.print(info->value);
  });
  serialEx.println();
}

void pulse() {
  static uint8_t ch[] = {0, 31, 63, 94, 127, 158, 191, 222};
  static int8_t st[] = {1, 1, 1, 1, 1, 1, 1, 1};
  static auto last = millis();
  auto now = millis();
  if (now - last < 20) return;
  last = now;
  for (uint8_t i = 0; i < 8; ++i) {
    if (ch[i] == 0) { st[i] = 1; }
    if (ch[i] == 255) { st[i] = -1; }
    ch[i] += st[i];
  }
  PWM::set(0, ch[0], 0, ch[3]);
  PWM::set(1, ch[2], 0, ch[5]);
  PWM::set(2, ch[4], 0, ch[7]);
  PWM::set(3, ch[6], 0, ch[1]);
  PWM::update();
}

void do_pulse(Args) {
  idle_fn = pulse;
}

void set_rgb(Args args) {
  idle_fn = nullptr;
  uint8_t zone = atoi(args.next());
  uint8_t red = atoi(args.next());
  uint8_t green = atoi(args.next());
  uint8_t blue = atoi(args.next());
  PWM::set(zone, red, green, blue);
  PWM::update();
}

void set_scale(Args args) {
  uint8_t scale = atoi(args.next());
  Timer2::prescaler::write(scale);
}
