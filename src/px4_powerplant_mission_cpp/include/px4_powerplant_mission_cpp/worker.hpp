#pragma once
#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>

namespace powerplant {
// One running job and at most one pending replacement. Cancellation never joins
// on the ROS thread.
template <class Result> class LatestWorker {
public:
  using Token = std::shared_ptr<std::atomic_bool>;
  using Work = std::function<Result(const Token &)>;
  LatestWorker() : thread_([this] { run(); }) {}
  ~LatestWorker() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
      invalidate();
    }
    ready_.notify_one();
    thread_.join();
  }
  void cancel() {
    std::lock_guard<std::mutex> lock(mutex_);
    invalidate();
  }
  void submit(Work work) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      invalidate();
      token_ = std::make_shared<std::atomic_bool>(false);
      pending_ = std::move(work);
    }
    ready_.notify_one();
  }
  std::optional<Result> take() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto out = std::move(result_);
    result_.reset();
    return out;
  }

private:
  std::mutex mutex_;
  std::condition_variable ready_;
  bool stop_ = false;
  Token token_;
  std::optional<Work> pending_;
  std::optional<Result> result_;
  std::thread thread_;
  void invalidate() {
    if (token_)
      token_->store(true);
    pending_.reset();
    result_.reset();
  }
  void run() {
    for (;;) {
      Work work;
      Token token;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait(lock, [this] { return stop_ || pending_.has_value(); });
        if (stop_)
          return;
        work = std::move(*pending_);
        pending_.reset();
        token = token_;
      }
      try {
        auto value = work(token);
        std::lock_guard<std::mutex> lock(mutex_);
        if (!stop_ && !token->load() && token == token_)
          result_ = std::move(value);
      } catch (...) { // Work functions turn application failures into typed
                      // results; cancellation is discarded.
      }
    }
  }
};
} // namespace powerplant
