#pragma once

#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

// Fixed-size thread pool backed by a single mutex-protected task queue.
//
//   run_and_wait(tasks)     - run an arbitrary batch of closures/funcs,
//                             blocks until all of them finish.
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

    template <typename F>
        requires std::invocable<F&, std::size_t, std::size_t>
    void parallel_for(F&& fn, std::size_t count = 1);

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

template <typename F>
    requires std::invocable<F&, std::size_t, std::size_t>
void ThreadPool::parallel_for(F&& fn, std::size_t count)
{
    if (count == 0) return;

    std::size_t chunks_count = std::min(count, thread_count());
    std::size_t chunk_size = (count + chunks_count - 1) / chunks_count;

    std::vector<std::move_only_function<void()>> tasks;
    tasks.reserve(chunks_count);
    for (std::size_t i = 0; i < chunks_count; ++i) {
        std::size_t begin = i * chunk_size;
        std::size_t end = std::min(begin + chunk_size, count);
        if (begin >= end) {
            break;
        }
        // Intentionally keep fn as an lvalue:
        // it is invoked by multiple tasks, so forwarding/moving it would be incorrect.
        tasks.emplace_back([&fn, begin, end] { fn(begin, end); });
    }

    run_and_wait(std::move(tasks));
}

}  // namespace ts
