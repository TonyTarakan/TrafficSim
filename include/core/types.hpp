#pragma once

#include <cstdint>
#include <functional>
#include <limits>

namespace ts {

template <typename Tag>
class [[nodiscard]] StrongId {
public:
    constexpr explicit StrongId(std::uint32_t v = 0) noexcept : value(v) {}
    constexpr auto operator<=>(const StrongId&) const noexcept = default;
    [[nodiscard]] constexpr std::uint32_t get() const noexcept { return value; }

private:
    std::uint32_t value;
};

using NodeId = StrongId<struct NodeTag>;
using EdgeId = StrongId<struct EdgeTag>;
using LaneId = StrongId<struct LaneTag>;
using VehicleId = StrongId<struct VehicleTag>;
using TrafficLightId = StrongId<struct TrafficLightTag>;

inline constexpr NodeId kInvalidNode{std::numeric_limits<std::uint32_t>::max()};
inline constexpr EdgeId kInvalidEdge{std::numeric_limits<std::uint32_t>::max()};
inline constexpr LaneId kInvalidLane{std::numeric_limits<std::uint32_t>::max()};

}  // namespace ts

namespace std {

template <typename Tag>
struct hash<ts::StrongId<Tag>> {
    constexpr size_t operator()(ts::StrongId<Tag> id) const noexcept { return hash<std::uint32_t>{}(id.get()); }
};

}  // namespace std
