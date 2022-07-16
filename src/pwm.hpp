#pragma once

#include <stdint.h>

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
public:
  using TYPE = typename PORT::TYPE;

private:
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

  template <typename F>
  static void for_each_channel(F&& proc) {
    for (uint8_t i = 0; i < ZONES * CHANNELS; ++i) {
      proc(&channels[i], i / CHANNELS, i % CHANNELS);
    }
  }

  template <typename F>
  static void for_each_event(F&& proc) {
    for (auto event = front; event != nullptr; event = event->next) {
      proc(event);
    }
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
    TYPE output = PORT::MASK;
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
    static auto next = front;
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
