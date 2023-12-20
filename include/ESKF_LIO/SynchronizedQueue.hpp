#ifndef ESKF_LIO_SYNCHRONIZED_QUEUE_HPP_
#define ESKF_LIO_SYNCHRONIZED_QUEUE_HPP_

#include <mutex>
#include <optional>
#include <queue>
#include <vector>

template<typename T>
class SynchronizedQueue
{
public:
  SynchronizedQueue() {}
  ~SynchronizedQueue() {}

  void push(const T & data)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    queue_.push(data);
  }

  void push(T && data)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    queue_.push(std::move(data));
  }

  std::optional<T> pop()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      return std::nullopt;
    }
    T data = std::move(queue_.front());
    queue_.pop();
    return data;
  }

  std::deque<T> popAll()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      return {};
    }

    std::deque<T> data;
    while (!queue_.empty()) {
      data.push_back(queue_.front());
      queue_.pop();
    }
    return data;
  }

private:
  std::queue<T> queue_;
  std::mutex mutex_;
};

#endif // ESKF_LIO_SYNCHRONIZED_QUEUE_HPP_
