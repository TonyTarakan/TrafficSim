#include <gtest/gtest.h>

#include "core/vehicle.hpp"

using namespace ts;

TEST(Vehicle, DefaultConstruction)
{
    Vehicle v;
    EXPECT_EQ(v.id, 0u);
    EXPECT_EQ(v.type, VehicleType::Car);
    EXPECT_FLOAT_EQ(v.speed, 0.f);
}

TEST(Vehicle, CanSetFields)
{
    Vehicle v;
    v.id = 42;
    v.type = VehicleType::Truck;
    v.position = {.x = 10.f, .y = 20.f};
    v.speed = 15.f;

    EXPECT_EQ(v.id, 42u);
    EXPECT_EQ(v.type, VehicleType::Truck);
    EXPECT_FLOAT_EQ(v.position.x, 10.f);
    EXPECT_FLOAT_EQ(v.position.y, 20.f);
    EXPECT_FLOAT_EQ(v.speed, 15.f);
}
