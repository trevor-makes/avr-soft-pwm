#pragma once

#include <stdint.h>

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

template <typename PORT, typename OCR, uint8_t ZONES, uint8_t CHANNELS_PER_ZONE>
class Controller {
  static constexpr auto NUM_CHANNELS = ZONES * CHANNELS_PER_ZONE;
  static constexpr auto NUM_EVENTS = NUM_CHANNELS + 1;

  using TYPE = typename PORT::TYPE;
  using EVENTS = Events<PORT, NUM_EVENTS>;

  Channel<TYPE> channels_[NUM_CHANNELS];
  DoubleBuffer<EVENTS> events_;
  typename EVENTS::iterator isr_iter_;
  uint16_t counter_ = 0;
  bool dirty_ = false;

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

  template <typename F>
  void for_each_channel(F&& proc) {
    for (uint8_t i = 0; i < ZONES * CHANNELS_PER_ZONE; ++i) {
      uint8_t zone = i / CHANNELS_PER_ZONE;
      uint8_t channel = i % CHANNELS_PER_ZONE;
      proc(&channels_[i], zone, channel);
    }
  }

  typename EVENTS::iterator event_iter() {
    return events_.front().iter();
  }

  uint16_t get_count() {
    return counter_;
  }

  bool can_update() {
    return dirty_ == false;
  }

  void update() {
    // TODO triple buffer would let us update more than once per cycle and keep the latest
    if (!dirty_) {
      // Update events in back buffer
      events_.back().update(channels_);
      dirty_ = true;
    }
  }

  void isr() {
    // When the end of the event queue is reached...
    if (!isr_iter_.has_next()) {
      ++counter_;
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
