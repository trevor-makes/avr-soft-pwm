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

template <typename TYPE>
struct Channel {
  TYPE pin;
  uint8_t duty;
};

template <typename PORT, uint8_t SIZE>
class Events {
  using TYPE = typename PORT::TYPE;

  struct Event {
    TYPE pins;
    uint8_t delta;
  };

  Event events_[SIZE];
  uint8_t count_ = 0;

public:
  // TODO could take a channel iterator instead
  template <uint8_t N>
  void update(Channel<TYPE> (&channels)[N]) {
    count_ = 0;
    insert({0, 0});
    for (auto& channel : channels) {
      insert(channel);
    }
    compile();
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

protected:
  void insert(Channel<TYPE> const& channel) {
    // 100% duty cycle channels never turn off, so they don't need an event
    if (channel.duty == 255) {
      return;
    }
    // Find index at which to insert, sorted by ascending duty cycle
    uint8_t index = 0;
    for (; index < count_; ++index) {
      auto& event = events_[index];
      if (event.delta == channel.duty) {
        // Coalesce channels with the same duty cycle; return without insertion
        event.pins |= channel.pin;
        return;
      } else if (event.delta > channel.duty) {
        // Found the index to insert at, so break loop early
        break;
      }
    }
    // Make room for inserted event
    auto& event = events_[index];
    memmove(&event + 1, &event, (count_ - index) * sizeof(Event));
    // Reset inserted event
    event.pins = channel.pin;
    event.delta = channel.duty;
    // Update size for inserted event
    ++count_;
  }

  // Pre-compute timer delta and accumulate toggled pins
  void compile() {
    TYPE output = PORT::MASK;
    uint8_t elapsed = 0;
    for (uint8_t index = 0; index < count_; ++index) {
      auto& event = events_[index];
      // Clear output pins toggled by event
      output &= ~event.pins;
      event.pins = output;
      if (index + 1 < count_) {
        // Compute delta to next event
        auto& next = events_[index + 1];
        event.delta = next.delta - elapsed - 1; // OCR = N - 1 sets timer to N cycles
        elapsed = next.delta;
      } else {
        // Set final delta for 255 cycles total
        event.delta = 254 - elapsed; // OCR = 254 sets timer to 255 cycles
        break;
      }
    }
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

template <typename PORT, typename OCR, uint8_t ZONES, uint8_t CHANNELS_PER_ZONE, uint8_t KEYFRAMES>
class Controller {
  static constexpr auto NUM_CHANNELS = ZONES * CHANNELS_PER_ZONE;
  static constexpr auto NUM_EVENTS = NUM_CHANNELS + 1;

  using TYPE = typename PORT::TYPE;
  using EVENTS = Events<PORT, NUM_EVENTS>;

  Channel<TYPE> channels_[NUM_CHANNELS];
  Keyframes<KEYFRAMES> keyframes_[NUM_CHANNELS];
  DoubleBuffer<EVENTS> events_;
  typename EVENTS::iterator isr_iter_;
  uint16_t period_ = 0;
  volatile uint16_t counter_ = 0;
  volatile bool dirty_ = false;

public:
  void init() {
    PORT::config_output();
  }

  template <uint8_t C = 0, typename T>
  void config(uint8_t zone, T index) {
    static_assert(C < CHANNELS_PER_ZONE, "too many channel parameters");
    channels_[zone * CHANNELS_PER_ZONE + C].pin = TYPE(1) << index;
    channels_[zone * CHANNELS_PER_ZONE + C].duty = 0;
  }

  template <uint8_t C = 0, typename T, typename... Var>
  void config(uint8_t zone, T index, const Var... var) {
    config<C>(zone, index);
    config<C + 1, Var...>(zone, var...);
  }

  template <uint8_t C = 0, typename T>
  void set(uint8_t zone, T duty) {
    static_assert(C < CHANNELS_PER_ZONE, "too many channel parameters");
    channels_[zone * CHANNELS_PER_ZONE + C].duty = duty;
  }

  template <uint8_t C = 0, typename T, typename... Var>
  void set(uint8_t zone, T duty, const Var... var) {
    set<C>(zone, duty);
    set<C + 1, Var...>(zone, var...);
  }

  template <uint8_t C = 0, typename T>
  void get(uint8_t zone, T& duty) {
    static_assert(C < CHANNELS_PER_ZONE, "too many channel parameters");
    duty = channels_[zone * CHANNELS_PER_ZONE + C].duty;
  }

  template <uint8_t C = 0, typename T, typename... Var>
  void get(uint8_t zone, T& duty, Var&... var) {
    get<C>(zone, duty);
    get<C + 1, Var...>(zone, var...);
  }

  void set_period(uint16_t period) {
    // Prevent ISR from reading counter/period while we write them
    auto guard = ISRGuard();
    counter_ = 0;
    period_ = period;
  }

  void clear_keyframes(uint8_t zone) {
    for (uint8_t i = 0; i < CHANNELS_PER_ZONE; ++i) {
      keyframes_[zone * CHANNELS_PER_ZONE + i].clear();
    }
  }

  template <uint8_t C = 0, typename T>
  void add_keyframe(uint8_t zone, uint16_t time, T duty) {
    static_assert(C < CHANNELS_PER_ZONE, "too many channel parameters");
    keyframes_[zone * CHANNELS_PER_ZONE + C].insert(time % period_, duty);
  }

  template <uint8_t C = 0, typename T, typename... Var>
  void add_keyframe(uint8_t zone, uint16_t time, T duty, const Var... var) {
    add_keyframe<C>(zone, time, duty);
    add_keyframe<C + 1, Var...>(zone, time, var...);
  }

  typename EVENTS::iterator event_iter() {
    return events_.front().iter();
  }

  bool can_update() {
    return dirty_ == false;
  }

  void update() {
    if (can_update()) {
      // Update events in back buffer
      events_.back().update(channels_);
      dirty_ = true;
    }
  }

  void animate() {
    if (can_update()) {
      uint16_t counter;
      { // Prevent ISR from writing counter while we read it
        auto guard = ISRGuard();
        counter = counter_;
      }
      for (uint8_t i = 0; i < NUM_CHANNELS; ++i) {
        channels_[i].duty = keyframes_[i].evaluate(counter, period_);
      }
      update();
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
