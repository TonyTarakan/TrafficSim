#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <numeric>
#include <stdexcept>
#include <thread>
#include <vector>

#include "concurrency/thread_pool.hpp"

using namespace ts;
using namespace std::chrono_literals;

TEST(ThreadPool, DefaultConstructorPicksAtLeastOneThread)
{
    ThreadPool pool;
    EXPECT_GE(pool.thread_count(), 1u);
}

TEST(ThreadPool, ExplicitThreadCountIsRespected)
{
    ThreadPool pool(4);
    EXPECT_EQ(pool.thread_count(), 4u);
}

TEST(ThreadPool, ExplicitZeroFallsBackToAutoDetect)
{
    // thread_count == 0 means "auto-detect" (see .cpp), not "no workers" --
    // a genuinely empty pool would hang every parallel_for() call forever.
    ThreadPool pool(0);
    EXPECT_GE(pool.thread_count(), 1u);
}

TEST(ThreadPool, ParallelForCoversEveryIndexExactlyOnce)
{
    constexpr std::size_t kCount = 1000;
    ThreadPool pool(4);

    std::vector<int> touched(kCount, 0);
    pool.parallel_for(
        [&](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) {
                touched[i]++;
            }
        },
        kCount);

    // Every index hit exactly once -- no gaps between chunks, no overlaps.
    EXPECT_EQ(std::accumulate(touched.begin(), touched.end(), 0), static_cast<int>(kCount));
    EXPECT_TRUE(std::ranges::all_of(touched, [](int t) { return t == 1; }));
}

TEST(ThreadPool, ParallelForWithZeroCountDoesNothing)
{
    ThreadPool pool(4);
    std::atomic<int> calls{0};

    pool.parallel_for([&](std::size_t, std::size_t) { calls.fetch_add(1); }, 0);

    EXPECT_EQ(calls.load(), 0);
}

TEST(ThreadPool, ParallelForWithFewerItemsThanThreads)
{
    // count < thread_count() exercises chunks_count = min(count, thread_count())
    // -- most of the pool should simply stay idle, not error out.
    constexpr std::size_t kCount = 3;
    ThreadPool pool(8);

    std::vector<int> touched(kCount, 0);
    pool.parallel_for(
        [&](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) {
                touched[i]++;
            }
        },
        kCount);

    EXPECT_TRUE(std::ranges::all_of(touched, [](int t) { return t == 1; }));
}

TEST(ThreadPool, ParallelForWithSingleWorkerThread)
{
    // Degenerates to one chunk covering everything -- effectively sequential.
    constexpr std::size_t kCount = 200;
    ThreadPool pool(1);

    std::vector<int> touched(kCount, 0);
    pool.parallel_for(
        [&](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) {
                touched[i]++;
            }
        },
        kCount);

    EXPECT_TRUE(std::ranges::all_of(touched, [](int t) { return t == 1; }));
}

TEST(ThreadPool, ParallelForBlocksUntilChunksFinish)
{
    // parallel_for() is synchronous: by the time it returns, every chunk's
    // writes must already be visible to the calling thread (no separate
    // "wait" call needed, unlike a fire-and-forget submit()).
    constexpr std::size_t kCount = 500;
    ThreadPool pool(4);

    std::vector<int> values(kCount, 0);
    pool.parallel_for(
        [&](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) {
                values[i] = static_cast<int>(i) * 2;
            }
        },
        kCount);

    for (std::size_t i = 0; i < kCount; ++i) {
        EXPECT_EQ(values[i], static_cast<int>(i) * 2);
    }
}

TEST(ThreadPool, ChunksActuallyRunConcurrently)
{
    // Four chunks that each sleep 100ms should finish in ~100ms total on a
    // 4+ worker pool, not ~400ms -- otherwise parallel_for() is just a
    // sequential loop with extra steps.
    ThreadPool pool(4);

    auto start = std::chrono::steady_clock::now();
    pool.parallel_for([](std::size_t, std::size_t) { std::this_thread::sleep_for(100ms); }, 4);
    auto elapsed = std::chrono::steady_clock::now() - start;

    EXPECT_LT(elapsed, 250ms);  // generous margin over the ideal 100ms
}

TEST(ThreadPool, SameCountAsThreadsGivesOneItemPerChunk)
{
    // count == thread_count() -- every worker gets exactly one index, the
    // boundary case between "some workers idle" and "workers share chunks".
    ThreadPool pool(4);

    std::vector<int> touched(4, 0);
    pool.parallel_for(
        [&](std::size_t begin, std::size_t end) {
            EXPECT_EQ(end - begin, 1u);
            for (std::size_t i = begin; i < end; ++i) {
                touched[i]++;
            }
        },
        4);

    EXPECT_TRUE(std::ranges::all_of(touched, [](int t) { return t == 1; }));
}

TEST(ThreadPool, MultipleSequentialCallsReuseTheSamePool)
{
    // Mirrors how SimEngine::tick() actually uses the pool: several
    // parallel_for() calls in a row on one long-lived ThreadPool, each
    // depending on the previous one having fully finished.
    constexpr std::size_t kCount = 100;
    ThreadPool pool(4);

    std::vector<int> stage_a(kCount, 0);
    std::vector<int> stage_b(kCount, 0);

    pool.parallel_for(
        [&](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) stage_a[i] = static_cast<int>(i);
        },
        kCount);

    pool.parallel_for(
        [&](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) stage_b[i] = stage_a[i] * 10;
        },
        kCount);

    for (std::size_t i = 0; i < kCount; ++i) {
        EXPECT_EQ(stage_b[i], static_cast<int>(i) * 10);
    }
}

TEST(ThreadPool, ConstructDestructWithoutAnyWorkIsSafe)
{
    ThreadPool pool(4);
    // No parallel_for() calls at all -- the destructor still has to stop
    // and join every worker cleanly.
    SUCCEED();
}
