#pragma once

#include <array>
#include <atomic>
#include <cstdint>

// Buffer from a sim thread to a render thread.
// Uses a lock-free 'atomic exchange shared spare' pattern

namespace ts {

template <typename T>
class TripleBuffer {
public:
    // Writer thread only.
    [[nodiscard]]
    T& back() & noexcept
    {
        return bufs_[write_idx_];
    }

    // Writer thread only.
    void publish() noexcept
    {
        std::uint32_t published_code = write_idx_ | kDirtyBit;  // mark as filled
        std::uint32_t prev_wr_code = spare_idx_.exchange(published_code, std::memory_order_acq_rel);
        write_idx_ = prev_wr_code & kIndexMask;  // new place to write
    }

    // Reader thread only.
    // Returns:
    //  - true if we have new data
    //  - false if no new data available
    [[nodiscard]]
    bool consume() noexcept
    {
        std::uint32_t curr_rd_code = spare_idx_.load(std::memory_order_acquire);
        if ((curr_rd_code & kDirtyBit) == 0) {
            return false;  // nothing new since last consume()
        }

        std::uint32_t prev_rd_code = spare_idx_.exchange(read_idx_, std::memory_order_acq_rel);
        read_idx_ = prev_rd_code & kIndexMask;

        return true;
    }

    // Reader thread only.
    // Stable between consume() calls.
    [[nodiscard]]
    const T& front() const& noexcept
    {
        return bufs_[read_idx_];
    }

private:
    static constexpr std::uint32_t kDirtyBit = 0x8000'0000u;   // Senior bit as flag/switch
    static constexpr std::uint32_t kIndexMask = 0x0000'0003u;  // Two bits mask
    // static constexpr auto kCacheAlign = std::hardware_destructive_interference_size;
    static constexpr auto kCacheAlign = 64;  // to avoid [-Winterference-size] warning

    alignas(kCacheAlign) std::array<T, 3> bufs_{};

    alignas(kCacheAlign) std::uint32_t write_idx_ = 0;              // writer-private(back), never touched by reader
    alignas(kCacheAlign) std::uint32_t read_idx_{1};                // reader-private(front), never touched by writer
    alignas(kCacheAlign) std::atomic<std::uint32_t> spare_idx_{2};  // the shared spare slot (clean, no dirty bit)
};

}  // namespace ts
