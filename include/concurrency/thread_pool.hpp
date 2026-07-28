#pragma once

#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

// Fixed-size thread pool backed by a single mutex-protected task queue.
//
//   run_and_wait(tasks) - run an arbitrary batch of closures, block until all of them finish.
//   parallel_for(count, fn) - splits [0, count) into up to num_threads()
//                             chunks and runs fn(begin, end) for each chunk.

namespace ts {

class ThreadPool {
public:
    explicit ThreadPool(std::size_t thread_count = 0);
    ~ThreadPool();

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;
    ThreadPool(ThreadPool&&) = delete;
    ThreadPool& operator=(ThreadPool&&) = delete;

    void parallel_for(const std::function<void(std::size_t, std::size_t)>& fn, std::size_t count = 1);

    [[nodiscard]] std::size_t thread_count() const noexcept { return workers_.size(); }

private:
    void run_and_wait(std::vector<std::move_only_function<void()>> tasks);
    void worker_loop();

    std::vector<std::thread> workers_;

    std::mutex mutex_;
    std::condition_variable task_available_cv_;
    std::vector<std::move_only_function<void()>> task_queue_;

    bool stop_{false};
};

}  // namespace ts
