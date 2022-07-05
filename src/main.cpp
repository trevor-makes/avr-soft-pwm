// Copyright (c) 2022 Trevor Makes

#include "uIO.hpp"
#include "uCLI.hpp"

#include "pwm.hpp"

#include <Arduino.h>
//#include <avr/interrupt.h>

// This tutorial is a good resource to mention
//https://raw.githubusercontent.com/abcminiuser/avr-tutorials/master/Timers/Output/Timers.pdf

using uCLI::StreamEx;
using uCLI::Args;
StreamEx serialEx(Serial);
uCLI::CLI<> serialCli(serialEx);

uIO_REG(TCCR2A)
uIO_REG(TCCR2B)
uIO_REG(TIMSK2)
uIO_REG(OCR2A)

using RegCOM2A = uIO::RightAlign<RegTCCR2A::Mask<0xC0>>;
using RegCS2 = RegTCCR2B::Mask<0x07>;
using RegWGM2 = uIO::BitExtend<RegTCCR2B::Mask<0x08>, RegTCCR2A::Mask<0x03>>;
using BitOCIE2A = RegTIMSK2::Bit<OCIE2A>;

const auto ZONES = 6;
const auto CHANNELS = 3;
using PWMPins = uIO::WordExtend<
  uIO::WordExtend<uIO::PortB, uIO::PortC>,
  uIO::WordExtend<uIO::PortD::Mask<0xFC>>>;
using PWM = SoftwarePWM<PWMPins, RegOCR2A, ZONES, CHANNELS>;

// Hook PWM routine into timer 2 compare interrupt
ISR(TIMER2_COMPA_vect) {
  PWM::isr();
}

void setup() {
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

  // Configure hardware timer 2
  RegWGM2::write(2); // enable CTC mode
  RegCOM2A::write(0); // disconnect OC2A output
  BitOCIE2A::set(); // enable output compare interrupt
  RegCS2::write(4); // clk/64 prescaler

  // Establish serial connection with computer
  Serial.begin(9600);
  while (!Serial) {}
}

void debug_pwm(uCLI::Args);
void do_pulse(uCLI::Args);
void set_rgb(uCLI::Args);
void set_scale(uCLI::Args);

uCLI::IdleFn idle_fn = nullptr;

void loop() {
  static const uCLI::Command commands[] = {
    { "pulse", do_pulse },
    { "set", set_rgb },
    { "debug", debug_pwm },
    { "scale", set_scale },
  };

  serialCli.run_once(commands, idle_fn);
}

void debug_pwm(uCLI::Args) {
  PWM::for_each<>([](PWMEvent<PWM::TYPE>* event) {
    // Print the value of each output bit this frame
    // TODO fix the leading zeros so bits line up vertically
    serialEx.print(event->bits, BIN);
    serialEx.print(" for ");
    // Print duty cycle of this frame (ticks * 100 / 255)
    // NOTE [>> 8 or / 256] is much faster and close enough to [/ 255]
    uint16_t ticks = event->delay + 1;
    serialEx.print((ticks * 100 + 127) >> 8);
    serialEx.println("%");
  });
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

void do_pulse(uCLI::Args) {
  idle_fn = pulse;
}

void set_rgb(uCLI::Args args) {
  idle_fn = nullptr;
  uint8_t zone = atoi(args.next());
  uint8_t red = atoi(args.next());
  uint8_t green = atoi(args.next());
  uint8_t blue = atoi(args.next());
  PWM::set(zone, red, green, blue);
  PWM::update();
}

void set_scale(uCLI::Args args) {
  uint8_t scale = atoi(args.next());
  RegCS2::write(scale);
}
