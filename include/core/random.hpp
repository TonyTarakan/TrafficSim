#pragma once

#include <random>

namespace ts {

template <typename T>
    requires std::integral<T> || std::floating_point<T>
[[nodiscard]] inline T generate_rand(T from, T to)
{
    thread_local std::mt19937 rng{std::random_device{}()};

    if constexpr (std::integral<T>) {
        std::uniform_int_distribution<T> dist{from, to};
        return dist(rng);
    }
    else {
        std::uniform_real_distribution<T> dist{from, to};
        return dist(rng);
    }
}

}  // namespace ts
