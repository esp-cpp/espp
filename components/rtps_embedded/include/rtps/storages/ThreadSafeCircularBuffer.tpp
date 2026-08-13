
#ifndef RTPS_THREADSAFECIRCULARBUFFER_TPP
#define RTPS_THREADSAFECIRCULARBUFFER_TPP

namespace rtps {

template <typename T, uint16_t SIZE>
bool ThreadSafeCircularBuffer<T, SIZE>::moveElementIntoBuffer(T &&elem) {
  std::lock_guard<std::mutex> lock(m_mutex);
  if (isFull()) {
#ifdef RTPS_STORAGE_DYNAMIC
    grow(); // host: grow instead of dropping the incoming element
#else
    m_insertion_failures++;
    return false;
#endif
  }
  m_buffer[m_head] = std::move(elem);
  incrementHead();
  return true;
}

template <typename T, uint16_t SIZE>
bool ThreadSafeCircularBuffer<T, SIZE>::copyElementIntoBuffer(const T &elem) {
  std::lock_guard<std::mutex> lock(m_mutex);
  if (isFull()) {
#ifdef RTPS_STORAGE_DYNAMIC
    grow(); // host: grow instead of dropping the incoming element
#else
    m_insertion_failures++;
    return false;
#endif
  }
  m_buffer[m_head] = elem;
  incrementHead();
  return true;
}

template <typename T, uint16_t SIZE>
bool ThreadSafeCircularBuffer<T, SIZE>::moveFirstInto(T &hull) {
  std::lock_guard<std::mutex> lock(m_mutex);
  if (m_head != m_tail) {
    hull = std::move(m_buffer[m_tail]);
    incrementTail();
    return true;
  } else {
    return false;
  }
}

template <typename T, uint16_t SIZE> bool ThreadSafeCircularBuffer<T, SIZE>::peakFirst(T &hull) {
  std::lock_guard<std::mutex> lock(m_mutex);
  if (m_head != m_tail) {
    hull = m_buffer[m_tail];
    return true;
  } else {
    return false;
  }
}

template <typename T, uint16_t SIZE> uint32_t ThreadSafeCircularBuffer<T, SIZE>::numElements() {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_num_elements;
}

template <typename T, uint16_t SIZE>
uint32_t ThreadSafeCircularBuffer<T, SIZE>::insertionFailures() {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_insertion_failures;
}

template <typename T, uint16_t SIZE> void ThreadSafeCircularBuffer<T, SIZE>::clear() {
  std::lock_guard<std::mutex> lock(m_mutex);
  m_head = m_tail;
  m_num_elements = 0;
}

template <typename T, uint16_t SIZE> bool ThreadSafeCircularBuffer<T, SIZE>::isFull() const {
  auto it = m_head;
  incrementIterator(it);
  return it == m_tail;
}

template <typename T, uint16_t SIZE>
inline void ThreadSafeCircularBuffer<T, SIZE>::incrementIterator(uint16_t &iterator) const {
  ++iterator;
  if (iterator >= m_buffer.size()) {
    iterator = 0;
  }
}

template <typename T, uint16_t SIZE>
inline void ThreadSafeCircularBuffer<T, SIZE>::incrementTail() {
  incrementIterator(m_tail);
  m_num_elements--;
}

template <typename T, uint16_t SIZE>
inline void ThreadSafeCircularBuffer<T, SIZE>::incrementHead() {
  incrementIterator(m_head);
  m_num_elements++;
  if (m_head == m_tail) {
    incrementTail();
  }
}

#ifdef RTPS_STORAGE_DYNAMIC
template <typename T, uint16_t SIZE> void ThreadSafeCircularBuffer<T, SIZE>::grow() {
  // Caller holds m_mutex. Double the ring capacity, then re-linearize the live
  // elements so the head/tail indices remain valid after the append-only
  // resize. The live range [m_tail, m_head) is move-assigned into the freshly
  // grown region [oldCap, ...) in logical order (move-assignment only, so the
  // element type need not be move-constructible). Element count is preserved,
  // so m_num_elements is left unchanged.
  const std::size_t oldCap = m_buffer.size();
  m_buffer.ensureSize(oldCap * 2);
  std::size_t dst = oldCap;
  uint16_t src = m_tail;
  while (src != m_head) {
    m_buffer[dst] = std::move(m_buffer[src]);
    ++dst;
    ++src;
    if (src >= oldCap) {
      src = 0; // wrap at the OLD capacity
    }
  }
  m_tail = static_cast<uint16_t>(oldCap);
  m_head = static_cast<uint16_t>(dst);
}
#endif
} // namespace rtps

#endif // RTPS_THREADSAFECIRCULARBUFFER_TPP
