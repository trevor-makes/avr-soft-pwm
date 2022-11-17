// Copyright (c) 2022 Trevor Makes

#include "core/io.hpp"
#include "core/cli.hpp"
#include "core/pwm.hpp"

#include <Arduino.h>
#include <EEPROM.h>

using core::cli::Args;
core::serial::StreamEx serialEx(Serial);
core::cli::CLI<> serialCli(serialEx);

struct Serializer {
  static void save_byte(uint16_t address, uint8_t data) {
    EEPROM.update(address, data);
  }

  template <typename T>
  static void save(uint16_t address, const T& data) {
    EEPROM.put(address, data);
  }

  static uint8_t load_byte(uint16_t address) {
    return EEPROM.read(address);
  }

  template <typename T>
  static void load(uint16_t address, T& data) {
    EEPROM.get(address, data);
  }

  static uint16_t get_size() {
    return EEPROM.length();
  }
};

#ifdef __AVR_ATmega328P__
struct PWMTimer {
  // uIO types for Timer2 registers
  CORE_REG(TCCR2A)
  CORE_REG(TCCR2B)
  CORE_REG(TIMSK2)
  CORE_REG(OCR2A)

  // Aliases for Timer2 bitfields
  using RegCOM2A = core::io::RightAlign<RegTCCR2A::Mask<0xC0>>;
  using RegCS2 = RegTCCR2B::Mask<0x07>;
  using RegWGM2 = core::io::BitExtend<RegTCCR2B::Bit<WGM22>, RegTCCR2A::Mask<0x03>>;
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

CORE_PORT(B)
CORE_PORT(C)
CORE_PORT(D)

// Use Rx pin for ISR measurement pulse (Serial must be disabled)
using MeasurePin = PortD::Bit<0>;

// PWM Controller pin mapping
// PortD is most significant (byte 2), PortB is least (byte 0)
// [x x x x x x x x | D7 D6 D5 D4 D3 D2 x x | x x C5 C4 C3 C2 C1 C0 | x x B5 B4 B3 B2 B1 B0]
using PWMPins = core::io::WordExtend<PortD::Mask<0xFC>, PortC::Mask<0x3F>, PortB::Mask<0x3F>>;

constexpr const uint8_t N_ZONES = 6;
constexpr const uint8_t N_PER_ZONE = 3;
constexpr const uint8_t N_KEYFRAMES = 8;
core::pwm::Controller<PWMPins, PWMTimer, N_ZONES, N_PER_ZONE, N_KEYFRAMES> pwm;

void config_pins() {
  PWMPins::config_output();

  // Configure mapping from zone/channel to bit within port register
  //  Blue 5 - D2 |         | x
  //   Red 5 - D3 |         | x
  // Green 5 - D4 |         | C5 - Green 0
  //  Blue 2 - D5 |         | C4 -   Red 0
  //   Red 2 - D6 | Arduino | C3 -  Blue 0
  // Green 2 - D7 |   NANO  | C2 - Green 1
  //  Blue 4 - B0 |         | C1 -   Red 1
  //   Red 4 - B1 |         | C0 -  Blue 1
  // Green 4 - B2 |   ___   | x
  //  Blue 3 - B3 |  |USB|  | x
  //   Red 3 - B4 |__|___|__| B5 - Green 3
  constexpr auto B = 0, C = 8, D = 16;
  pwm.config_pins(0, C + 4, C + 5, C + 3); // r0, g0, b0
  pwm.config_pins(1, C + 1, C + 2, C + 0); // r1, g1, b1
  pwm.config_pins(2, D + 6, D + 7, D + 5); // r2, g2, b2
  pwm.config_pins(3, B + 4, B + 5, B + 3); // r3, g3, b3
  pwm.config_pins(4, B + 1, B + 2, B + 0); // r4, g4, b4
  pwm.config_pins(5, D + 3, D + 4, D + 2); // r5, g5, b5
}

// Hook PWM routine into timer 2 compare interrupt
ISR(TIMER2_COMPA_vect) {
  pwm.isr();
}

// Timer0 overflow used by Arduino for micros/millis/etc
struct MicrosTimer {
  CORE_REG(TIMSK0)

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
void set_white(Args);
void set_sweep(Args);

void setup() {
  config_pins();

  // Load from EEPROM if valid, otherwise use default
  if (!pwm.load<Serializer>(0)) {
    set_default({});
  }

  PWMTimer::config();

  // OPTIONAL Disable Arduino micros interrupt (will break micros/millis/delay/etc!)
  // Software PWM will work without disabling this, but it causes tiny jitter observable with a scope
  // Feel free to remove this if micros is needed
  MicrosTimer::disable_isr();

  // Establish serial connection with computer
  Serial.begin(9600);
  while (!Serial) {}
}

void do_list(Args);
void do_save(Args);
void do_load(Args);
void do_clear(Args);
void config_channel(Args);
void set_keyframe(Args);
void set_period(Args);
void set_range(Args);
void measure_isr(Args);

constexpr const char* KEYFRAME_CMD = "keyframe";
constexpr const char* PERIOD_CMD = "period";
constexpr const char* RANGE_CMD = "range";

void loop() {
  static const core::cli::Command commands[] = {
    { "config", config_channel },
    { KEYFRAME_CMD, set_keyframe },
    { PERIOD_CMD, set_period },
    { RANGE_CMD, set_range },
    { "clear", do_clear },
    { "default", set_default },
    { "rainbow", set_rainbow },
    { "white", set_white },
    { "sweep", set_sweep },
    { "list", do_list },
    { "save", do_save },
    { "load", do_load },
    { "measure", measure_isr },
  };

  // Update PWM animation while waiting for CLI input
  serialCli.run_once(commands, []() { pwm.update(); });
}

uint8_t get_range(Args& args) {
  if (args.has_next()) {
    uint8_t range = atoi(args.next());
    pwm.set_range(range);
    return range;
  } else {
    return pwm.get_range();
  }
}

void set_default(Args) {
  pwm.clear_all();
  pwm.set_range(255);
  constexpr uint8_t TIME = 0;
  pwm.set_keyframe(0, TIME, 255, 0, 127);
  pwm.set_keyframe(1, TIME, 191, 0, 63);
  pwm.set_keyframe(2, TIME, 191, 31, 63);
  pwm.set_keyframe(3, TIME, 191, 63, 63);
  pwm.set_keyframe(4, TIME, 255, 63, 15);
  pwm.set_keyframe(5, TIME, 255, 31, 0);
}

void set_rainbow(Args args) {
  pwm.clear_all();
  // Range affects the period of the PWM signal; smaller range results in higher frequency
  uint8_t range = get_range(args);
  // 245 ticks is 1 second when range is 255
  // TODO maybe set_period/keyframe should take float seconds?
  uint16_t ticks = 98 * 255 / range; // 98 ticks is 0.4 seconds
  pwm.set_period(15 * ticks); // Loop every 6 seconds
  for (uint8_t i = 0; i < 6; ++i) {
    // Cycle from red to green to blue with 2 seconds between each
    // Offset each zone by 0.4 seconds
    pwm.set_keyframe(i, (0 + i) * ticks, range, 0, 0); // Red
    pwm.set_keyframe(i, (5 + i) * ticks, 0, range, 0); // Green
    pwm.set_keyframe(i, (10 + i) * ticks, 0, 0, range); // Blue
  }
}

void set_white(Args args) {
  pwm.clear_all();
  // Range affects the period of the PWM signal; smaller range results in higher frequency
  uint8_t range = get_range(args);
  for (uint8_t i = 0; i < 6; ++i) {
    // TODO adjustable color temperature, maybe lerp over [range/2, range]
    pwm.set_keyframe(i, 0, range, range >> 1, range >> 1);
  }
}

void set_sweep(Args args) {
  uint8_t red[6];
  uint8_t green[6];
  uint8_t blue[6];
  for (uint8_t i = 0; i < 6; ++i) {
    uint16_t time;
    pwm.get_keyframe(i, 0, time, red[i], green[i], blue[i]);
  }
  pwm.clear_all();
  for (uint8_t i = 0; i < 6; ++i) {
    pwm.set_keyframe(i, 0, red[i], green[i], blue[i]);
    pwm.set_keyframe(i, 50, 0, 0, 0);
    pwm.set_keyframe(i, 75 * (i + 1), 0, 0, 0);
    pwm.set_keyframe(i, 1 + 75 * (i + 1), red[i], green[i], blue[i]);
    pwm.set_keyframe(i, 75 * (i + 2), red[i], green[i], blue[i]);
    pwm.set_keyframe(i, 1 + 75 * (i + 2), 0, 0, 0);
    pwm.set_keyframe(i, 25 + 75 * (5 + 2), 0, 0, 0);
    pwm.set_keyframe(i, 75 + 75 * (5 + 2), red[i], green[i], blue[i]);
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

void set_range(Args args) {
  pwm.set_range(atoi(args.next()));
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
  print_list(RANGE_CMD, pwm.get_range());

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

void do_save(Args args) {
  uint8_t index = atoi(args.next());
  if (!pwm.save<Serializer>(index)) {
    serialEx.println("Unable to save");
  }
}

void do_load(Args args) {
  uint8_t index = atoi(args.next());
  if (!pwm.load<Serializer>(index)) {
    serialEx.println("Unable to load");
  }
}

void measure_isr(Args args) {
  serialEx.println("Connect scope to pin D0 (Rx) and single trigger on pulse width > 1 us to measure ISR");

  // Disable Rx on pin D0 so we can use it as a digital I/O
  Serial.end();
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
