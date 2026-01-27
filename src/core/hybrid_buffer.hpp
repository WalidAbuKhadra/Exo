#pragma once
#include <atomic>
#include <cstdint>
#include <memory>
#include <new>
#include <version>

namespace core {

#ifdef __cpp_lib_hardware_interference_size
constexpr std::size_t CACHE_LINE_SIZE = std::hardware_destructive_interference_size;
#else
constexpr std::size_t CACHE_LINE_SIZE = 64;
#endif

template <typename U>
struct CacheLinePadded {
  static_assert(sizeof(std::atomic<U>) <= CACHE_LINE_SIZE);

  alignas(CACHE_LINE_SIZE) std::atomic<U> value;

  char pad[CACHE_LINE_SIZE - sizeof(std::atomic<U>)];

  constexpr CacheLinePadded() noexcept : value(), pad{} {}
  constexpr CacheLinePadded(U v) noexcept : value(v), pad{} {}
};

template <typename T>
class HybridBuffer {
  struct State {
    uint8_t index;
    bool isNew;
  };

public:
  HybridBuffer() {
    for (int i = 0; i < 4; ++i) {
      m_buffers[i] = std::make_unique<T>();
    }
  }

  template <typename Function>
  void InitializeBuffer(Function fn) {
    for (int i = 0; i < 4; ++i) {
      if (m_buffers[i]) {
        fn(*m_buffers[i]);
      }
    }
  }

  T *GetWriteBuffer() {
    auto idx = m_writeIndex.value.load(std::memory_order_relaxed);
    return m_buffers[idx].get();
  }

  void Publish() {
    int currentWriteIdx = m_writeIndex.value.load(std::memory_order_relaxed);
    State newState{static_cast<uint8_t>(currentWriteIdx), true};
    State oldState = m_sharedState.value.exchange(newState, std::memory_order_acq_rel);
    m_writeIndex.value.store(oldState.index, std::memory_order_relaxed);
  }

  T *Fetch() {
    State current = m_sharedState.value.load(std::memory_order_acquire);

    while (current.isNew) {
      bool guiWaiting = m_guiWaiting.value.load(std::memory_order_acquire);
      uint8_t indexToRecycle;

      if (guiWaiting) {
        indexToRecycle = m_previewIndex.value.exchange(
            static_cast<uint8_t>(m_readIndex.value.load(std::memory_order_relaxed)), std::memory_order_acq_rel);
      } else {
        indexToRecycle = static_cast<uint8_t>(m_readIndex.value.load(std::memory_order_relaxed));
      }

      State desired{indexToRecycle, false};

      if (m_sharedState.value.compare_exchange_weak(current, desired, std::memory_order_acq_rel,
                                                    std::memory_order_acquire)) {

        if (guiWaiting) {
          m_guiWaiting.value.store(false, std::memory_order_release);
        }

        m_readIndex.value.store(current.index, std::memory_order_relaxed);
        return m_buffers[m_readIndex.value.load(std::memory_order_relaxed)].get();
      }

      if (guiWaiting) {
        m_previewIndex.value.store(indexToRecycle, std::memory_order_release);
      }
    }

    return nullptr;
  }

  T *GetPeekIfNew() {
    if (!m_guiWaiting.value.load(std::memory_order_acquire)) {
      uint8_t idx = m_previewIndex.value.load(std::memory_order_acquire);
      return m_buffers[idx].get();
    }
    return nullptr;
  }

  void RequestNewPeek() { m_guiWaiting.value.store(true, std::memory_order_release); }

private:
  std::unique_ptr<T> m_buffers[4];

  CacheLinePadded<int> m_writeIndex{0};
  CacheLinePadded<int> m_readIndex{1};
  CacheLinePadded<State> m_sharedState{State{2, false}};
  CacheLinePadded<uint8_t> m_previewIndex{3};
  CacheLinePadded<bool> m_guiWaiting{false};
};

} // namespace core