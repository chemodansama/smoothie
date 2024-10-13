#pragma once

#include <Splines.h>

namespace smoothie
{

inline SplineLib::Vec2f operator+(SplineLib::Vec2f a, SplineLib::Vec2f b)
{
    return { a.x + b.x, a.y + b.y };
}

inline SplineLib::Vec2f operator-(SplineLib::Vec2f a, SplineLib::Vec2f b)
{
    return { a.x - b.x, a.y - b.y };
}

inline SplineLib::Vec2f operator*(float s, SplineLib::Vec2f a)
{
    return { s * a.x, s * a.y };
}

}
