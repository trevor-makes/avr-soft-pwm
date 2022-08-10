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

namespace uPWM {

template <typename TYPE, uint8_t SIZE>
class Events {
  struct Event {
    TYPE pins;
    uint8_t delta;
  };

  Event events_[SIZE];
  uint8_t count_ = 0;

public:
  void reset() {
    // Reserve first event for initial pin state
    events_[0].pins = 0;
    count_ = 1;
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
};

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

  bool get(uint8_t index, uint16_t& time, uint8_t& value) {
    if (index < count_) {
      time = frames_[index].time;
      value = frames_[index].value;
      return true;
    } else {
      return false;
    }
  }

  bool insert(uint16_t time, uint8_t value) {
    if (count_ == N) {
      return false;
    }
    // Find insertion index sorted by ascending time
    uint8_t index = 0;
    for (; index < count_; ++index) {
      if (time == frames_[index].time) {
        // Overwrite old keyframe
        frames_[index].value = value;
        return true;
      } else if (time < frames_[index].time) {
        break;
      }
    }
    // Move later keyframes back to make room and insert
    memmove(&frames_[index+1], &frames_[index], (count_ - index) * sizeof(Keyframe));
    frames_[index].time = time;
    frames_[index].value = value;
    ++count_;
    return true;
  }

  uint8_t evaluate(uint16_t time, uint16_t period) {
    if (count_ == 0) {
      // Return zero/black if no keyframes
      return 0;
    } else if (count_ == 1) {
      // Return exact value if only one keyframe
      return frames_[0].value;
    }

    // Find index of first frame later than given time
    uint8_t index = 0;
    for (; index < count_; ++index) {
      auto const& frame_time = frames_[index].time;
      if (time == frame_time) {
        // Return exact value if timestamps match
        return frames_[index].value;
      } else if (time < frame_time) {
        break;
      }
    }

    // Lerp between previous and next keyframes
    // v(t) = (v0 * (t1 - t) + v1 * (t - t0)) / (t1 - t0)
    uint16_t delta; // t1 - t0
    uint32_t prev; // v0 * (t1 - t)
    uint32_t next; // v1 * (t - t0)
    if (index == 0) {
      // Previous keyframe [index - 1] wraps-around to [count_ - 1]
      // time < frames_[index].time < frames_[count_ - 1].time
      delta = frames_[index].time + (period - frames_[count_ - 1].time);
      prev = uint32_t(frames_[count_ - 1].value) * (frames_[index].time - time);
      next = uint32_t(frames_[index].value) * (time + (period - frames_[count_ - 1].time));
    } else if (index == count_) {
      // Following keyframe [index] wraps-around to [0]
      // frames_[0].time < frames_[index - 1].time < time
      delta = frames_[0].time + (period - frames_[index - 1].time);
      prev = uint32_t(frames_[index - 1].value) * (frames_[0].time + (period - time));
      next = uint32_t(frames_[0].value) * (time - frames_[index - 1].time);
    } else {
      // frames_[index - 1].time < time < frames_[index].time
      delta = frames_[index].time - frames_[index - 1].time;
      prev = uint32_t(frames_[index - 1].value) * (frames_[index].time - time);
      next = uint32_t(frames_[index].value) * (time - frames_[index - 1].time);
    }
    return (prev + next) / delta;
  }
};

template <typename PORT, typename OCR, uint8_t N_CHANNELS, uint8_t N_KEYFRAMES>
class Controller {
  using TYPE = typename PORT::TYPE;
  using EVENTS = Events<TYPE, N_CHANNELS + 1>;

  TYPE pins_[N_CHANNELS];
  Keyframes<N_KEYFRAMES> keyframes_[N_CHANNELS];
  DoubleBuffer<EVENTS> events_;
  typename EVENTS::iterator isr_iter_;
  uint16_t period_ = 0;
  volatile uint16_t counter_ = 0;
  volatile bool dirty_ = false;

public:
  template <uint8_t C = 0, typename T>
  void config_pins(uint8_t channel, T bit_index) {
    pins_[channel + C] = TYPE(1) << bit_index;
  }

  template <uint8_t C = 0, typename T, typename... Var>
  void config_pins(uint8_t channel, T bit_index, const Var... var) {
    // Index by multiple of parameter count
    if (C == 0) {
      channel *= 1 + sizeof...(Var);
    }
    config_pins<C>(channel, bit_index);
    config_pins<C + 1, Var...>(channel, var...);
  }

  template <uint8_t C = 0, typename T>
  void set_keyframe(uint8_t channel, uint16_t time, T value) {
    keyframes_[channel + C].insert(time % period_, value);
  }

  template <uint8_t C = 0, typename T, typename... Var>
  void set_keyframe(uint8_t channel, uint16_t time, T value, const Var... var) {
    // Index by multiple of parameter count
    if (C == 0) {
      channel *= 1 + sizeof...(Var);
    }
    set_keyframe<C>(channel, time, value);
    set_keyframe<C + 1, Var...>(channel, time, var...);
  } 

  bool get_keyframe(uint8_t channel, uint8_t index, uint16_t& time, uint8_t& value) {
    return keyframes_[channel].get(index, time, value);
  }

  void set_period(uint16_t period) {
    // Prevent ISR from reading counter/period while we write them
    auto guard = ISRGuard();
    counter_ = 0;
    period_ = period;
  }

  template <uint8_t CHANNELS_PER_ZONE = 1>
  void clear_zone(uint8_t zone) {
    for (uint8_t i = 0; i < CHANNELS_PER_ZONE; ++i) {
      keyframes_[zone * CHANNELS_PER_ZONE + i].clear();
    }
  }

  void clear_all() {
    for (auto& keyframe : keyframes_) {
      keyframe.clear();
    }
  }

  typename EVENTS::iterator event_iter() {
    return events_.front().iter();
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
      for (uint8_t i = 0; i < N_CHANNELS; ++i) {
        events.insert(pins_[i], keyframes_[i].evaluate(counter, period_));
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
      OCR::write(next->delta);
      PORT::write(next->pins);
    } else {
      // If iterator is empty just clear GPIO pins
      PORT::clear();
    }
  }
};

} // namespace uPWM
