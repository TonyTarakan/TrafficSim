#include <gtest/gtest.h>

#include "core/vec2d.hpp"

using namespace ts;

TEST(Vec2D, Addition)
{
    Vec2D a{.x = 1.f, .y = 2.f};
    Vec2D b{.x = 3.f, .y = 4.f};
    Vec2D c = a + b;
    EXPECT_FLOAT_EQ(c.x, 4.f);
    EXPECT_FLOAT_EQ(c.y, 6.f);
}

TEST(Vec2D, Subtraction)
{
    Vec2D a{.x = 5.f, .y = 5.f};
    Vec2D b{.x = 2.f, .y = 1.f};
    Vec2D c = a - b;
    EXPECT_FLOAT_EQ(c.x, 3.f);
    EXPECT_FLOAT_EQ(c.y, 4.f);
}

TEST(Vec2D, ScalarMultiply)
{
    Vec2D a{.x = 2.f, .y = 3.f};
    Vec2D c = a * 2.f;
    EXPECT_FLOAT_EQ(c.x, 4.f);
    EXPECT_FLOAT_EQ(c.y, 6.f);
}

TEST(Vec2D, DotProduct)
{
    Vec2D a{.x = 1.f, .y = 0.f};
    Vec2D b{.x = 0.f, .y = 1.f};
    EXPECT_FLOAT_EQ(a.dot_prod(b), 0.f);

    Vec2D c{.x = 3.f, .y = 4.f};
    EXPECT_FLOAT_EQ(c.dot_prod(c), 25.f);
}

TEST(Vec2D, LengthSquared)
{
    Vec2D a{.x = 3.f, .y = 4.f};
    EXPECT_FLOAT_EQ(a.length_sq(), 25.f);
}
