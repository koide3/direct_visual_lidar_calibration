#pragma once

#include <deque>
#include <mutex>
#include <atomic>
#include <optional>
#include <condition_variable>

namespace vlcal {

/**
 * @brief Simple thread-safe queue with mutex
 */
template <typename T>
class ConcurrentQueue {
public:
  ConcurrentQueue() : end_of_data(false), num_pushed(0) {}
  ~ConcurrentQueue() {}

  void submit_end_of_data() {
    end_of_data = true;
    cond.notify_all();
  }

  bool get_end_of_data() const { return end_of_data; }

  size_t size() const {
    std::lock_guard<std::mutex> lock(mutex);
    return queue.size();
  }

  size_t size_total() const { return num_pushed; }

  void push(const T& data) {
    std::lock_guard<std::mutex> lock(mutex);
    queue.emplace_back(data);
    cond.notify_one();
    num_pushed++;
  }

  void push(const T&& data) {
    std::lock_guard<std::mutex> lock(mutex);
    queue.emplace_back(data);
    cond.notify_one();
    num_pushed++;
  }

  std::optional<T> pop() {
    std::lock_guard<std::mutex> lock(mutex);
    if (queue.empty()) {
      return std::nullopt;
    }

    const T data = queue.front();
    queue.pop_front();
    return data;
  }

  std::optional<T> pop_lock() {
    std::unique_lock<std::mutex> lock(mutex);

    std::optional<T> data;
    cond.wait(lock, [this, &data] {
      if (queue.empty()) {
        return static_cast<bool>(end_of_data);
      }

      data = queue.front();
      queue.pop_front();
      return true;
    });

    return data;
  }

  std::deque<T> get_all_and_clear() {
    std::lock_guard<std::mutex> lock(mutex);
    std::deque<T> new_queue;
    queue.swap(new_queue);
    return new_queue;
  }

private:
  std::atomic_bool end_of_data;
  std::atomic_int num_pushed;

  std::condition_variable cond;
  mutable std::mutex mutex;
  std::deque<T> queue;
};

}  // namespace calibox
