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
constexpr const uint8_t PRESCALE_256 = 6;
using Timer2 = Timer<RegTCNT2, RegCS2, RegWGM2, BitTOIE2>;
using Timer2A = TimerCompare<RegOCR2A, RegCOM2A, BitOCIE2A>;

// PWM Controller pin mapping
constexpr const uint8_t PWM_ZONES = 6;
constexpr const uint8_t PWM_CHANNELS = 3;
using PWMPins = uIO::WordExtend<
  uIO::WordExtend<uIO::PortB::Mask<0x3F>, uIO::PortC::Mask<0x3F>>,
  uIO::WordExtend<uIO::PortD::Mask<0xFC>>>;
uPWM::Controller<PWMPins, Timer2A::value, PWM_ZONES, PWM_CHANNELS> pwm;

// Hook PWM routine into timer 2 compare interrupt
ISR(TIMER2_COMPA_vect) {
  pwm.isr();
}
#else
#error Need to provide configuration for current platform. See __AVR_ATmega328P__ configuration above.
#endif

void setup() {
  pwm.init();

  // Configure mapping from zone/channel to bit within port register
  //     7  6  5  4  3  2  1  0
  // B [ -  - g3 r3 b3 g4 r4 b4]
  // C [ -  - g0 r0 b0 g1 r1 b1]
  // D [g2 r2 b2 g5 r5 b5  -  -]
  constexpr auto B = 0, C = 8, D = 16;
  pwm.config(0, C + 4, C + 5, C + 3); // r0, g0, b0
  pwm.config(1, C + 1, C + 2, C + 0); // r1, g1, b1
  pwm.config(2, D + 6, D + 7, D + 5); // r2, g2, b2
  pwm.config(3, B + 4, B + 5, B + 3); // r3, g3, b3
  pwm.config(4, B + 1, B + 2, B + 0); // r4, g4, b4
  pwm.config(5, D + 3, D + 4, D + 2); // r5, g5, b5

  // Set RGB value (duty cycle) to default gradient
  pwm.set(0, 255, 0, 191);
  pwm.set(1, 191, 0, 63);
  pwm.set(2, 191, 31, 63);
  pwm.set(3, 191, 63, 63);
  pwm.set(4, 255, 63, 15);
  pwm.set(5, 255, 31, 0);

  // Compile PWM timing for new values and reprogram next cycle
  pwm.update();

#ifdef __AVR_ATmega328P__
  // Configure hardware timer
  Timer2::waveform::write(WAVEFORM_CTC); // enable CTC mode
  Timer2A::output_mode::write(OUTPUT_MODE_OFF); // disconnect OC2A output
  Timer2A::interrupt::set(); // enable compare interrupt
  // NOTE longest ISR measured was ~150 clocks, so use the next largest prescaler (timer must be slower than ISR)
  Timer2::prescaler::write(PRESCALE_256); // divide by 256
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
void config_channel(Args);
void set_channel(Args);
void set_prescaler(Args);
void measure_isr(Args);

uCLI::IdleFn idle_fn = nullptr;

void loop() {
  static const uCLI::Command commands[] = {
    { "pulse", do_pulse },
    { "config", config_channel },
    { "set", set_channel },
    { "list", do_list },
    { "debug", debug_pwm },
    { "measure", measure_isr },
    { "prescaler", set_prescaler },
  };

  serialCli.run_once(commands, idle_fn);
}

void measure_isr(Args) {
  serialEx.println("Measure pulse width on pin B5 (13 on Uno/Nano)");
  // Toggle pin B5 in an infinite loop to generate a 2 MHz square wave (16 MHz / 8 CPU cycles)
  // When an ISR runs it will interrupt the loop and the wave will be stuck high or low
  // Measure the pulse width minus 1/2 period (4 CPU cycles) to find the ISR duration
  // -| 8 |- CPU cycles
  //  |   |   |- ISR length -|     |- ISR length -|
  // _/-\_/-\_:______________/-\_/-:--------------\_
  for (;;) { // 8 CPU cycles per loop
    uIO::PinB5::set(); // SBI, 2 cycles
    __asm__("NOP\nNOP\n"); // 2 cycles (1 per NOP)
    uIO::PinB5::clear(); // CBI, 2 cycles
  } // RJMP, 2 cycles
}

void debug_pwm(Args) {
  auto iter = pwm.event_iter();
  while (iter.has_next()) {
    auto event = iter.next();
    // Print the value of each output bit this frame
    // TODO fix the leading zeros so bits line up vertically, like format_hex in uMon
    serialEx.print(event->pins, BIN);
    serialEx.print(" for ");
    // Print duty cycle of this frame (ticks * 100 / 255)
    // NOTE [/ 256] is within decimal precision of and much faster than [/ 255] (should compile to [>> 8])
    uint16_t ticks = event->delta + 1;
    serialEx.print((ticks * 100 + 127) / 256);
    serialEx.println("%");
  };
}

void do_list(Args args) {
  for (uint8_t zone = 0; zone < PWM_ZONES; ++zone) {
    uint8_t red, green, blue;
    pwm.get(zone, red, green, blue);
    serialEx.print(zone);
    serialEx.print(": ");
    serialEx.print(red);
    serialEx.print(", ");
    serialEx.print(green);
    serialEx.print(", ");
    serialEx.println(blue);
  }
}

template <uint8_t N>
class Keyframes {
  struct Keyframe {
    uint8_t value;
    uint16_t time;
  };

  Keyframe frames_[N];
  uint8_t count_ = 0;

public:
  void clear() { count_ = 0; }

  bool insert(uint16_t time, uint8_t value) {
    if (count_ == N) {
      return false;
    }
    uint8_t index = 0;
    for (; index < count_; ++index) {
      if (time < frames_[index].time) {
        break;
      }
    }
    memmove(&frames_[index+1], &frames_[index], (count_ - index) * sizeof(Keyframe));
    frames_[index].time = time;
    frames_[index].value = value;
    ++count_;
    return true;
  }

  uint8_t evaluate(uint16_t time, uint16_t period) {
    if (count_ == 0) {
      return 0;
    } else if (count_ == 1) {
      // NOTE the lerp logic should work with count_ == 1, so this special case isn't really necessary
      return frames_[0].value;
    }

    // Find index of first frame later than time
    uint8_t index = 0;
    for (; index < count_; ++index) {
      auto const& frame_time = frames_[index].time;
      if (time == frame_time) {
        // Just return the keyframe value if the time matches
        return frames_[index].value;
      } else if (time < frame_time) {
        break;
      }
    }

    // Lerp between previous and next keyframes
    // TODO refactor redundant logic
    if (index == 0) {
      // Previous keyframe wraps-around
      // [0] is next, [count_ - 1] is prev
      // time < [0] < [count_ - 1]
      uint16_t delta = frames_[0].time + (period - frames_[count_ - 1].time);
      uint32_t w0 = uint32_t(frames_[count_ - 1].value) * (frames_[0].time - time);
      uint32_t w1 = uint32_t(frames_[0].value) * (period - frames_[count_ - 1].time + time);
      return (w0 + w1) / delta;
    } else if (index == count_) {
      // Following keyframe wraps-around
      // [0] is next, [count_ - 1] is prev
      // [0] < [count_ - 1] < time
      uint16_t delta = frames_[0].time + (period - frames_[count_ - 1].time);
      uint32_t w0 = uint32_t(frames_[count_ - 1].value) * (period - time + frames_[0].time);
      uint32_t w1 = uint32_t(frames_[0].value) * (time - frames_[count_ - 1].time);
      return (w0 + w1) / delta;
    } else {
      // [index - 1] < time < [index]
      uint16_t delta = frames_[index].time - frames_[index - 1].time;
      uint32_t w0 = uint32_t(frames_[index - 1].value) * (frames_[index].time - time);
      uint32_t w1 = uint32_t(frames_[index].value) * (time - frames_[index - 1].time);
      return (w0 + w1) / delta;
    }
  }
};

template <uint8_t ZONES, uint8_t CHANNELS_PER_ZONE, uint8_t FRAMES>
class Animation {
  static_assert(CHANNELS_PER_ZONE == 3, "hard coded for 3 channels for now...");
  static constexpr auto NUM_CHANNELS = ZONES * CHANNELS_PER_ZONE;

  Keyframes<FRAMES> frames_[NUM_CHANNELS];
  uint16_t last_count_;
  uint16_t timer_ = 0;
  uint16_t period_ = 0;

public:
  void set_period(uint16_t period) { period_ = period; }

  void clear() {
    for (auto& frame : frames_) {
      frame.clear();
    }
  }

  template <uint8_t C = 0, typename T>
  void insert(uint16_t time, uint8_t zone, T duty) {
    static_assert(C < CHANNELS_PER_ZONE, "too many channel parameters");
    frames_[zone * CHANNELS_PER_ZONE + C].insert(time, duty);
  }

  template <uint8_t C = 0, typename T, typename... Var>
  void insert(uint16_t time, uint8_t zone, T duty, const Var... var) {
    insert<C>(time, zone, duty);
    insert<C + 1, Var...>(time, zone, var...);
  }

  // the set() interface kind of sucks here, unless we put the keyframes directly into the Controller class?
  template <typename PORT, typename OCR>
  void animate(uPWM::Controller<PORT, OCR, ZONES, 3>& controller) {
    uint16_t count = controller.get_count();
    timer_ = (timer_ + uint16_t(count - last_count_)) % period_;
    last_count_ = count;

    for (uint8_t i = 0; i < ZONES; ++i) {
      uint8_t red = frames_[i * 3].evaluate(timer_, period_);
      uint8_t green = frames_[i * 3 + 1].evaluate(timer_, period_);
      uint8_t blue = frames_[i * 3 + 2].evaluate(timer_, period_);
      controller.set(i, red, green, blue);
    }

    controller.update();
  }
};

Animation<6, 3, 2> animation;

void pulse() {
  if (pwm.can_update()) {
    animation.animate(pwm);
  }
}

void do_pulse(Args) {
  idle_fn = pulse;

  animation.clear();
  animation.set_period(1000);
  for (uint8_t i = 0; i < 6; ++i) {
    animation.insert(0 + i * 100, i, 255, 0, 0);
    animation.insert(500 + i * 100, i, 0, 0, 255);
  }
}

// Reconfigure pin mapping
void config_channel(Args args) {
  uint8_t zone = atoi(args.next());
  uint8_t red = atoi(args.next());
  uint8_t green = atoi(args.next());
  uint8_t blue = atoi(args.next());
  pwm.config(zone, red, green, blue);
}

// Reprogram channel
void set_channel(Args args) {
  idle_fn = nullptr;
  uint8_t zone = atoi(args.next());
  uint8_t red = atoi(args.next());
  uint8_t green = atoi(args.next());
  uint8_t blue = atoi(args.next());
  pwm.set(zone, red, green, blue);
  pwm.update();
}

// Select PWM timer frequency
void set_prescaler(Args args) {
  uint8_t prescaler = atoi(args.next());
  Timer2::prescaler::write(prescaler);
}
