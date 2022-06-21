// Copyright (c) 2022 Trevor Makes

#include "uIO.hpp"
#include "uCLI.hpp"

#include <Arduino.h>
//#include <avr/interrupt.h>

// This tutorial is a good resource to mention
//https://raw.githubusercontent.com/abcminiuser/avr-tutorials/master/Timers/Output/Timers.pdf

// TODO variadic template?
template <typename Port0, typename Port1, typename Port2>
struct PortExtend {
  using TYPE = uint32_t;

  // Select write mode for all ports
  static inline void config_output() {
    Port2::config_output();
    Port1::config_output();
    Port0::config_output();
  }

  // Write 16-bit value to high and low ports
  static inline void write(TYPE value) {
    Port2::write((value >> 16) & 0xFF);
    Port1::write((value >> 8) & 0xFF);
    Port0::write(value & 0xFF);
  }
};

using uCLI::StreamEx;
using uCLI::Args;
StreamEx serialEx(Serial);
uCLI::CLI<> serialCli(serialEx);

template <typename TYPE>
struct PWMChannel {
  TYPE bit;
  uint8_t value;
};

template <typename TYPE>
struct PWMEvent {
  TYPE bits;
  uint8_t delay;
  PWMEvent* next;
};

template <typename PORT, typename OCR, uint8_t ZONES, uint8_t CHANNELS>
class SoftwarePWM {
  using TYPE = typename PORT::TYPE;

  static PWMChannel<TYPE> channels[ZONES * CHANNELS];
  static PWMEvent<TYPE> events1[ZONES * CHANNELS + 1];
  static PWMEvent<TYPE> events2[ZONES * CHANNELS + 1];
  static PWMEvent<TYPE>* front;
  static PWMEvent<TYPE>* back;
  static bool dirty;

public:
  template <uint8_t C = 0, typename T>
  static void config(uint8_t zone, T index) {
    static_assert(C < CHANNELS, "too many channel parameters");
    channels[zone * CHANNELS + C].bit = TYPE(1) << index;
    channels[zone * CHANNELS + C].value = 0;
  }

  template <uint8_t C = 0, typename T, typename... Var>
  static void config(uint8_t zone, T index, const Var... var) {
    config<C>(zone, index);
    config<C + 1, Var...>(zone, var...);
  }

  template <uint8_t C = 0, typename T>
  static void set(uint8_t zone, T val) {
    static_assert(C < CHANNELS, "too many channel parameters");
    channels[zone * CHANNELS + C].value = val;
  }

  template <uint8_t C = 0, typename T, typename... Var>
  static void set(uint8_t zone, T val, const Var... var) {
    set<C>(zone, val);
    set<C + 1, Var...>(zone, var...);
  }

  static void update() {
    // Populate back event queue by insertion sort
    back->bits = 0;
    back->delay = 0;
    back->next = nullptr;
    auto next_alloc = back + 1;
    for (auto& channel : channels) {
      if (channel.value == 255) {
        // Discard channels set to max value
        continue;
      }
      for (auto cursor = back; /*see break*/; cursor = cursor->next) {
        if (channel.value == cursor->delay) {
          // Coalesce channels with the same value into a single event
          cursor->bits |= channel.bit;
          break;
        } else if (cursor->next == nullptr || channel.value < cursor->next->delay) {
          // Allocate new event after this one
          auto next = cursor->next;
          auto alloc = next_alloc++;
          cursor->next = alloc;
          alloc->bits = channel.bit;
          alloc->delay = channel.value;
          alloc->next = next;
          break;
        }
      }
    }
    // Accumulate turned-off bits and delays between events
    //TYPE output = PORT::MASK; // TODO Port/Port16 don't export the underlying register mask
    TYPE output = ~0; // start with all bits set
    uint8_t elapsed = 0;
    for (auto cursor = back; /*see break*/; cursor = cursor->next) {
      // Turn off selected bits
      output &= ~cursor->bits;
      cursor->bits = output;
      if (cursor->next != nullptr) {
        // Compute delay to next event
        auto next_time = cursor->next->delay;
        cursor->delay = next_time - elapsed - 1; // OCR set to N - 1 delays N cycles
        elapsed = next_time;
      } else {
        // Delay 255 cycles in total
        cursor->delay = 254 - elapsed;
        break;
      }
    }
    // Swap back buffer in at the end of the next cycle
    dirty = true;
  }

  static void isr() {
    static PWMEvent<TYPE>* next = front;
    OCR::write(next->delay);
    PORT::write(next->bits);
    next = next->next;
    if (next == nullptr) {
      if (dirty) {
        // Swap front and back queues
        auto swap = front;
        front = back;
        back = swap;
        dirty = false;
      }
      next = front;
    }
  }

  static void debug(uCLI::Args) {
    for (auto cursor = front; cursor != nullptr; cursor = cursor->next) {
      // Print the value of each output bit this frame
      serialEx.print(cursor->bits, BIN);
      serialEx.print(" for ");
      // Print duty cycle of this frame (ticks * 100 / 255)
      // NOTE [>> 8 or / 256] is much faster and close enough to [/ 255]
      uint16_t ticks = cursor->delay + 1;
      serialEx.print((ticks * 100 + 127) >> 8);
      serialEx.println("%");
    }
  }
};

template <typename PORT, typename OCR, uint8_t ZONES, uint8_t CHANNELS>
PWMChannel<typename PORT::TYPE> SoftwarePWM<PORT, OCR, ZONES, CHANNELS>::channels[ZONES * CHANNELS];

template <typename PORT, typename OCR, uint8_t ZONES, uint8_t CHANNELS>
PWMEvent<typename PORT::TYPE> SoftwarePWM<PORT, OCR, ZONES, CHANNELS>::events1[ZONES * CHANNELS + 1];

template <typename PORT, typename OCR, uint8_t ZONES, uint8_t CHANNELS>
PWMEvent<typename PORT::TYPE> SoftwarePWM<PORT, OCR, ZONES, CHANNELS>::events2[ZONES * CHANNELS + 1];

template <typename PORT, typename OCR, uint8_t ZONES, uint8_t CHANNELS>
PWMEvent<typename PORT::TYPE>* SoftwarePWM<PORT, OCR, ZONES, CHANNELS>::front = events1;

template <typename PORT, typename OCR, uint8_t ZONES, uint8_t CHANNELS>
PWMEvent<typename PORT::TYPE>* SoftwarePWM<PORT, OCR, ZONES, CHANNELS>::back = events2;

template <typename PORT, typename OCR, uint8_t ZONES, uint8_t CHANNELS>
bool SoftwarePWM<PORT, OCR, ZONES, CHANNELS>::dirty = false;

uIO_REG(TCCR2A)
uIO_REG(TCCR2B)
uIO_REG(TIMSK2)
uIO_REG(OCR2A)

using RegCOM2A = RegTCCR2A::Mask<0xC0>; // TODO port shift
using RegCS2 = RegTCCR2B::Mask<0x07>;
using RegWGM2 = uIO::PortJoin<RegTCCR2A::Mask<0x03>, RegTCCR2B::Mask<0x08>>; // TODO port shift
using BitOCIE2A = RegTIMSK2::Bit<OCIE2A>;

using PWMPins = PortExtend<uIO::PortB, uIO::PortC, uIO::PortD::Mask<0xFC>>;
using PWM = SoftwarePWM<PWMPins, RegOCR2A, 6, 3>;

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

void do_pulse(uCLI::Args);
void set_rgb(uCLI::Args);
void set_scale(uCLI::Args);

uCLI::IdleFn idle_fn = nullptr;

void loop() {
  static const uCLI::Command commands[] = {
    { "pulse", do_pulse },
    { "set", set_rgb },
    { "debug", PWM::debug },
    { "scale", set_scale },
  };

  serialCli.run_once(commands, idle_fn);
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
