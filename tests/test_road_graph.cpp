#include <gtest/gtest.h>

#include "core/road_graph.hpp"

using namespace ts;

TEST(RoadNode, DefaultIsInvalid)
{
    RoadNode n;
    EXPECT_EQ(n.id, kInvalidNode);
    EXPECT_FLOAT_EQ(n.z, 0.f);
}

TEST(RoadNode, CanSetFields)
{
    RoadNode n;
    n.id = 5;
    n.pos = {.x = 10.f, .y = 20.f};
    n.z = 1.f;

    EXPECT_EQ(n.id, 5u);
    EXPECT_FLOAT_EQ(n.pos.x, 10.f);
    EXPECT_FLOAT_EQ(n.pos.y, 20.f);
    EXPECT_FLOAT_EQ(n.z, 1.f);
}

TEST(Lane, DefaultIsInvalid)
{
    Lane l;
    EXPECT_EQ(l.id, kInvalidLane);
    EXPECT_EQ(l.from, kInvalidNode);
    EXPECT_EQ(l.to, kInvalidNode);
}

TEST(Lane, DefaultSpeedLimitIsReasonable)
{
    Lane l;
    // ~50 km/h in m/s — default for a lane.
    EXPECT_NEAR(l.speed_limit, 13.9f, 0.1f);
}

TEST(Lane, DefaultsToSingleSublane)
{
    Lane l;
    EXPECT_EQ(l.num_sublanes, 1);
}

TEST(Lane, CanConfigureMultiLaneSegment)
{
    Lane l;
    l.id = 0;
    l.from = 1;
    l.to = 2;
    l.length = 150.f;
    l.speed_limit = 27.8f;  // ~100 km/h
    l.num_sublanes = 3;

    EXPECT_EQ(l.from, 1u);
    EXPECT_EQ(l.to, 2u);
    EXPECT_FLOAT_EQ(l.length, 150.f);
    EXPECT_EQ(l.num_sublanes, 3);
}
