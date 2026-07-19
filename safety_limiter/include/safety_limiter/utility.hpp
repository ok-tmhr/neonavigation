#pragma once

#include <vector>

namespace safety_limiter
{

class vec
{
public:
float c[2];
vec(const float x, const float y)
{
    c[0] = x;
    c[1] = y;
}
vec()
{
    c[0] = c[1] = 0.0;
}
float& operator[](const int& i)
{
    assert(i < 2);
    return c[i];
}
const float& operator[](const int& i) const
{
    assert(i < 2);
    return c[i];
}
vec operator-(const vec& a) const
{
    vec out = *this;
    out[0] -= a[0];
    out[1] -= a[1];
    return out;
}
float cross(const vec& a) const
{
    return (*this)[0] * a[1] - (*this)[1] * a[0];
}
float dot(const vec& a) const
{
    return (*this)[0] * a[0] + (*this)[1] * a[1];
}
float dist(const vec& a) const
{
    return std::hypot((*this)[0] - a[0], (*this)[1] - a[1]);
}
float dist_line(const vec& a, const vec& b) const
{
    return (b - a).cross((*this) - a) / b.dist(a);
}
float dist_linestrip(const vec& a, const vec& b) const
{
    if ((b - a).dot((*this) - a) <= 0)
    return this->dist(a);
    if ((a - b).dot((*this) - b) <= 0)
    return this->dist(b);
    return std::abs(this->dist_line(a, b));
}
};
class polygon
{
public:
std::vector<vec> v;
void move(const float& x, const float& y, const float& yaw)
{
    const float cos_v = cosf(yaw);
    const float sin_v = sinf(yaw);
    for (auto& p : v)
    {
    const auto tmp = p;
    p[0] = cos_v * tmp[0] - sin_v * tmp[1] + x;
    p[1] = sin_v * tmp[0] + cos_v * tmp[1] + y;
    }
}
bool inside(const vec& a) const
{
    int cn = 0;
    for (size_t i = 0; i < v.size() - 1; i++)
    {
    auto& v1 = v[i];
    auto& v2 = v[i + 1];
    if ((v1[1] <= a[1] && a[1] < v2[1]) ||
        (v2[1] <= a[1] && a[1] < v1[1]))
    {
        float lx;
        lx = v1[0] + (v2[0] - v1[0]) * (a[1] - v1[1]) / (v2[1] - v1[1]);
        if (a[0] < lx)
        cn++;
    }
    }
    return ((cn & 1) == 1);
}
float dist(const vec& a) const
{
    float dist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < v.size() - 1; i++)
    {
    auto& v1 = v[i];
    auto& v2 = v[i + 1];
    auto d = a.dist_linestrip(v1, v2);
    if (d < dist)
        dist = d;
    }
    return dist;
}
};
}