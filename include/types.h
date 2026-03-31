#pragma once

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>

namespace px4ctrl {

using clock = std::chrono::high_resolution_clock;

inline double timeDuration(const clock::time_point &start,
                           const clock::time_point &end) {
  return static_cast<double>(
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count());
}

inline double timePassed(const clock::time_point &start) {
  return timeDuration(start, clock::now());
}

inline double timePassedSeconds(const clock::time_point &start) {
  return timeDuration(start, clock::now()) / 1000.0;
}

inline long to_uint64(const clock::time_point &time) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             time.time_since_epoch())
      .count();
}

inline clock::time_point from_uint64(const long &time) {
  return clock::time_point(std::chrono::milliseconds(time));
}

template <typename T> using Callback = std::function<void(const T &)>;
class Observer;
using FuncUnobserve = std::function<void(const Observer *)>;

class Observer {
public:
  explicit Observer(FuncUnobserve func) : m_func(std::move(func)) {}

  void unobserve() { m_func(this); }

  ~Observer() { unobserve(); }

private:
  FuncUnobserve m_func;
};

template <typename T> class Observable {
public:
  inline const T &value() const { return m_data; }

  inline void post(const T &data) {
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_data = data;
    }

    for (auto it = m_callbacks.begin(); it != m_callbacks.end(); it++) {
      it->second(m_data);
    }
  }

  inline std::shared_ptr<Observer> observe(Callback<T> callback) {
    auto observer = std::make_shared<Observer>(
        std::bind(&Observable<T>::removeObserver, this, std::placeholders::_1));
    m_callbacks[observer.get()] = std::move(callback);
    return observer;
  }

private:
  friend class Observer;

  std::mutex m_mutex;
  T m_data{};
  std::map<Observer *, Callback<T>> m_callbacks;

  inline void removeObserver(const Observer *observer) {
    for (auto it = m_callbacks.begin(); it != m_callbacks.end(); it++) {
      if (it->first == observer) {
        m_callbacks.erase(it);
        return;
      }
    }
  }
};

template <typename T> using Px4Data = Observable<T>;
template <typename T> using Px4DataPtr = std::shared_ptr<Observable<T>>;
using Px4DataObserver = std::shared_ptr<Observer>;

} // namespace px4ctrl
