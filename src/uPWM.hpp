#pragma once

#include <stdint.h>

// TODO move to a utilities header somewhere?
template <typename T, uint8_t N = 2>
class DoubleBuffer {
  static_assert(N == 2, "TODO implement 3+ swapping");
  T buffers_[N];
  T* front_ = &buffers_[0];
  T* back_ = &buffers_[1];

public:
  T& front() { return *front_; }
  T& back() { return *back_; }

  void flip() {
    auto swap = front_;
    front_ = back_;
    back_ = swap;
  }
};

// Protect a block of code from interrupts
// AVR provides ATOMIC_BLOCK in util/atomic.h for the same purpose
class ISRGuard {
  uint8_t sreg;
public:
  ISRGuard() {
    sreg = SREG;
    __asm__ __volatile__ ("cli");
  }
  ~ISRGuard() {
    SREG = sreg;
  }
};

template <uint8_t I = 0, typename T1, typename T2, uint8_t N>
void copy_from_args(T1 (&into)[N], const T2 from) {
  static_assert(I < N, "Too many args");
  into[I] = from;
}

template <uint8_t I = 0, typename T1, typename T2, uint8_t N, typename... Args>
void copy_from_args(T1 (&into)[N], const T2 from, const Args... args) {
  copy_from_args<I>(into, from);
  copy_from_args<I + 1>(into, args...);
}

template <uint8_t I = 0, typename T1, typename T2, uint8_t N>
void copy_into_args(const T1 (&from)[N], T2& into) {
  static_assert(I < N, "Too many args");
  into = from[I];
}

template <uint8_t I = 0, typename T1, typename T2, uint8_t N, typename... Args>
void copy_into_args(const T1 (&from)[N], T2& into, Args&... args) {
  copy_into_args<I>(from, into);
  copy_into_args<I + 1>(from, args...);
}

namespace uPWM {

template <uint8_t N_PER_ZONE, uint8_t N_KEYFRAMES>
class Keyframes {
  struct Keyframe {
    uint8_t values[N_PER_ZONE];
    uint16_t time;
  };

  Keyframe frames_[N_KEYFRAMES];
  uint8_t count_ = 0;

public:
  void clear() { count_ = 0; }

  template <typename... Args>
  bool get(uint8_t index, uint16_t& time, Args&... args) {
    if (index < count_) {
      time = frames_[index].time;
      copy_into_args(frames_[index].values, args...);
      return true;
    } else {
      return false;
    }
  }

  template <typename... Args>
  bool insert(uint16_t time, const Args... args) {
    // Find insertion index sorted by ascending time
    uint8_t index = 0;
    for (; index < count_; ++index) {
      if (time == frames_[index].time) {
        // Overwrite old keyframe
        copy_from_args(frames_[index].values, args...);
        return true;
      } else if (time < frames_[index].time) {
        break;
      }
    }
    // Can't insert at max capacity
    if (count_ == N_KEYFRAMES) {
      return false;
    }
    // Move later keyframes back to make room and insert
    memmove(&frames_[index+1], &frames_[index], (count_ - index) * sizeof(Keyframe));
    frames_[index].time = time;
    copy_from_args(frames_[index].values, args...);
    ++count_;
    return true;
  }

  void evaluate(uint16_t time, uint16_t period, uint8_t (&values)[N_PER_ZONE]) {
    if (count_ == 0) {
      // Return zero/black if no keyframes
      memset(values, 0, sizeof(values));
      return;
    } else if (count_ == 1) {
      // Return exact value if only one keyframe
      memcpy(values, frames_[0].values, sizeof(values));
      return;
    }

    // Find index of first frame later than given time
    uint8_t index = 0;
    for (; index < count_; ++index) {
      auto const& frame_time = frames_[index].time;
      if (time == frame_time) {
        // Return exact value if timestamps match
        memcpy(values, frames_[index].values, sizeof(values));
        return;
      } else if (time < frame_time) {
        break;
      }
    }

    // Pre-compute lerp weights from previous to next keyframe
    // v(t) = (v0 * (t1 - t) + v1 * (t - t0)) / (t1 - t0)
    uint16_t delta; // t1 - t0
    uint32_t prev_w; // t1 - t
    uint32_t next_w; // t - t0
    uint8_t prev_i, next_i;
    if (index == 0) {
      // Previous keyframe [index - 1] wraps-around to [count_ - 1]
      next_i = index;
      prev_i = count_ - 1;
      // time < frames_[index].time < frames_[count_ - 1].time
      delta = frames_[next_i].time + (period - frames_[prev_i].time);
      prev_w = frames_[next_i].time - time;
      next_w = time + (period - frames_[prev_i].time);
    } else if (index == count_) {
      // Following keyframe [index] wraps-around to [0]
      next_i = 0;
      prev_i = index - 1;
      // frames_[0].time < frames_[index - 1].time < time
      delta = frames_[next_i].time + (period - frames_[prev_i].time);
      prev_w = frames_[next_i].time + (period - time);
      next_w = time - frames_[prev_i].time;
    } else {
      next_i = index;
      prev_i = index - 1;
      // frames_[index - 1].time < time < frames_[index].time
      delta = frames_[next_i].time - frames_[prev_i].time;
      prev_w = frames_[next_i].time - time;
      next_w = time - frames_[prev_i].time;
    }

    // Evaluate lerp over each color channel
    for (uint8_t i = 0; i < N_PER_ZONE; ++i) {
      uint32_t prev = prev_w * frames_[prev_i].values[i]; // v0 * (t1 - t)
      uint32_t next = next_w * frames_[next_i].values[i]; // v1 * (t - t0)
      values[i] = (prev + next) / delta;
    }
  }
};

template <typename TYPE, uint8_t N_EVENTS>
class Events {
  struct Event {
    TYPE pins;
    uint8_t delta;
  };

  Event events_[N_EVENTS];
  uint8_t count_ = 0;

public:
  void reset() {
    // Reserve first event for initial pin state
    events_[0].pins = 0;
    count_ = 1;
  }

  void insert(TYPE pin, uint8_t value) {
    // Store the initial state of all pins in event 0
    if (value > 0) {
      events_[0].pins |= pin;
    }
    // 0% and 100% duty cycle channels never change state, so they don't need events
    if (value == 0 || value == 255) {
      return;
    }
    // Find index at which to insert, sorted by ascending value
    uint8_t index = 1;
    for (; index < count_; ++index) {
      auto& event = events_[index];
      if (event.delta == value) {
        // Event at this time already exists; combine pins and return
        event.pins |= pin;
        return;
      } else if (event.delta > value) {
        // Found the index to insert at, so break loop early
        break;
      }
    }
    // Make room for inserted event
    auto& event = events_[index];
    memmove(&event + 1, &event, (count_ - index) * sizeof(Event));
    // Reset inserted event
    event.pins = pin;
    event.delta = value;
    // Update size for inserted event
    ++count_;
  }

  // Pre-compute timer delta and accumulate toggled pins
  void compile() {
    TYPE output = events_[0].pins;
    uint8_t elapsed = 0;
    for (uint8_t index = 1; index < count_; ++index) {
      auto& event = events_[index];
      auto& prev = events_[index - 1];
      // Clear output pins toggled by event
      output &= ~event.pins;
      event.pins = output;
      // Compute timer delta between events
      // NOTE subtract 1 because the timer will delay at least 1 cycle
      prev.delta = event.delta - elapsed - 1;
      elapsed = event.delta;
    }
    // Set final delta for 255 cycles from first event
    // NOTE setting timer to 254 delays for 255 cycles
    events_[count_ - 1].delta = 254 - elapsed;
  }

  class iterator {
    Events* events_;
    uint8_t index_ = 0;

  public:
    iterator(): events_{nullptr} {}
    iterator(Events& events): events_{&events} {}

    iterator& operator=(iterator const& copy) {
      events_ = copy.events_;
      index_ = copy.index_;
      return *this;
    }

    bool has_next() {
      return events_ && index_ < events_->count_;
    }

    Event* next() {
      if (has_next()) {
        return &events_->events_[index_++];
      } else {
        return nullptr;
      }
    }
  };

  iterator iter() {
    return iterator(*this);
  }
};

template <typename GPIO, typename TIMER, uint8_t N_ZONES, uint8_t N_PER_ZONE, uint8_t N_KEYFRAMES>
class Controller {
  static constexpr auto N_CHANNELS = N_ZONES * N_PER_ZONE;
  static constexpr auto N_MAX_EVENTS = N_CHANNELS + 1;

  using TYPE = typename GPIO::TYPE;
  using EVENTS = Events<TYPE, N_MAX_EVENTS>;

  TYPE pins_[N_CHANNELS];
  Keyframes<N_PER_ZONE, N_KEYFRAMES> keyframes_[N_ZONES];
  DoubleBuffer<EVENTS> events_;
  typename EVENTS::iterator isr_iter_;
  uint16_t period_ = 0;
  volatile uint16_t counter_ = 0;
  volatile bool dirty_ = false;

public:
  template <uint8_t I = 0, typename T>
  void config_pins(uint8_t zone, T bit_index) {
    static_assert(I < N_PER_ZONE, "Too many args");
    pins_[zone * N_PER_ZONE + I] = TYPE(1) << bit_index;
  }

  template <uint8_t I = 0, typename T, typename... Args>
  void config_pins(uint8_t zone, T bit_index, const Args... var) {
    config_pins<I>(zone, bit_index);
    config_pins<I + 1, Args...>(zone, var...);
  }

  template <typename... Args>
  void set_keyframe(uint8_t zone, uint16_t time, const Args... args) {
    keyframes_[zone].insert(time % period_, args...);
  }

  template <typename... Args>
  bool get_keyframe(uint8_t zone, uint8_t index, uint16_t& time, Args&... args) {
    return keyframes_[zone].get(index, time, args...);
  }

  constexpr static uint8_t PWM_VERSION = '1';
  constexpr static uint16_t SAVE_SIZE = 6 + sizeof(keyframes_);

  template <typename STORE>
  bool save(uint8_t index) {
    if ((index + 1) * SAVE_SIZE > STORE::get_size()) return false;
    uint16_t base = index * SAVE_SIZE;
    STORE::save_byte(base, PWM_VERSION);
    STORE::save_byte(base + 1, N_ZONES);
    STORE::save_byte(base + 2, N_PER_ZONE);
    STORE::save_byte(base + 3, N_KEYFRAMES);
    STORE::save(base + 4, period_);
    STORE::save(base + 6, keyframes_);
    return true;
  }

  template <typename STORE>
  bool load(uint8_t index) {
    if ((index + 1) * SAVE_SIZE > STORE::get_size()) return false;
    uint16_t base = index * SAVE_SIZE;
    if (STORE::load_byte(base) != PWM_VERSION) return false;
    if (STORE::load_byte(base + 1) != N_ZONES) return false;
    if (STORE::load_byte(base + 2) != N_PER_ZONE) return false;
    if (STORE::load_byte(base + 3) != N_KEYFRAMES) return false;
    STORE::load(base + 4, period_);
    STORE::load(base + 6, keyframes_);
    return true;
  }

  void set_period(uint16_t period) {
    // Prevent ISR from reading counter/period while we write them
    auto guard = ISRGuard();
    counter_ = 0;
    period_ = period;
  }

  uint16_t get_period() {
    return period_;
  }

  void clear_zone(uint8_t zone) {
    keyframes_[zone].clear();
  }

  void clear_all() {
    for (auto& keyframe : keyframes_) {
      keyframe.clear();
    }
  }

  bool can_update() {
    return dirty_ == false;
  }

  void update() {
    if (can_update()) {
      uint16_t counter;
      { // Prevent ISR from writing counter while we read it
        auto guard = ISRGuard();
        counter = counter_;
      }
      // Recompile PWM event list with updated values
      auto& events = events_.back();
      events.reset();
      for (uint8_t zone = 0; zone < N_ZONES; ++zone) {
        uint8_t values[N_PER_ZONE];
        keyframes_[zone].evaluate(counter, period_, values);
        for (uint8_t i = 0; i < N_PER_ZONE; ++i) {
          events.insert(pins_[zone * N_PER_ZONE + i], values[i]);
        }
      }
      events.compile();
      // Swap event buffers when the PWM cycle resets
      dirty_ = true;
    }
  }

  void isr() {
    // When the end of the event queue is reached...
    if (!isr_iter_.has_next()) {
      // Increment counter up to period
      // NOTE %= is really slow so use subtraction instead
      ++counter_;
      if (counter_ >= period_) {
        counter_ -= period_;
      }
      // Flip back buffer if it was updated
      if (dirty_) {
        events_.flip();
        dirty_ = false;
      }
      // Reset iterator to start of event queue
      isr_iter_ = events_.front().iter();
    }
    // Handle the next event in the queue
    if (auto next = isr_iter_.next()) {
      // Update timer and GPIO registers according to event
      TIMER::set_delay(next->delta);
      GPIO::write(next->pins);
    } else {
      // If iterator is empty just clear GPIO pins
      GPIO::clear();
    }
  }
};

} // namespace uPWM
