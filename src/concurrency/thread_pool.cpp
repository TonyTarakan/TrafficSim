#include "concurrency/thread_pool.hpp"

#include <algorithm>
#include <latch>

namespace ts {

ThreadPool::ThreadPool(std::size_t thread_count)
{
    if (thread_count == 0) {
        thread_count = std::max(1u, std::thread::hardware_concurrency());
    }
    workers_.reserve(thread_count);
    for (std::size_t i = 0; i < thread_count; ++i) {
        workers_.emplace_back([this] { worker_loop(); });
    }
}

ThreadPool::~ThreadPool()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
    }
    task_available_cv_.notify_all();
    for (auto& t : workers_) {
        t.join();
    }
}

void ThreadPool::run_and_wait(std::vector<std::move_only_function<void()>> tasks)
{
    if (tasks.empty()) return;

    auto latch = std::make_shared<std::latch>(tasks.size());
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& t : tasks) {
            task_queue_.emplace_back([latch, task = std::move(t)]() mutable {
                task();
                latch->count_down();
            });
        }
    }
    task_available_cv_.notify_all();
    latch->wait();
}

void ThreadPool::parallel_for(const std::function<void(std::size_t, std::size_t)>& fn, std::size_t count)
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
        tasks.emplace_back([&fn, begin, end] { fn(begin, end); });
    }

    run_and_wait(std::move(tasks));
}

void ThreadPool::worker_loop()
{
    while (true) {
        std::move_only_function<void()> task;

        {
            std::unique_lock<std::mutex> lock(mutex_);
            task_available_cv_.wait(lock, [this] { return stop_ || !task_queue_.empty(); });

            if (stop_ && task_queue_.empty()) return;

            task = std::move(task_queue_.back());
            task_queue_.pop_back();
        }

        task();
    }
}

}  // namespace ts
