#pragma once

#include <CDT.h>

namespace smoothie
{

inline CDT::V2d<float> operator*(const CDT::V2d<float> &left, const CDT::V2d<float> &right)
{
    return CDT::V2d<float>::make(left.x * right.x, left.y * right.y);
}

inline CDT::V2d<float> operator*(const CDT::V2d<float> &left, float right)
{
    return CDT::V2d<float>::make(left.x * right, left.y * right);
}

inline CDT::V2d<float> operator/(const CDT::V2d<float> &left, float right)
{
    return CDT::V2d<float>::make(left.x / right, left.y / right);
}

inline CDT::V2d<float> operator+(const CDT::V2d<float> &left, const CDT::V2d<float> &right)
{
    return CDT::V2d<float>::make(left.x + right.x, left.y + right.y);
}

inline CDT::V2d<float> operator-(const CDT::V2d<float> &left, const CDT::V2d<float> &right)
{
    return CDT::V2d<float>::make(left.x - right.x, left.y - right.y);
}

}
