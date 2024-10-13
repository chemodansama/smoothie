#pragma once

#include <cassert>
#include <iterator>
#include <limits>
#include <map>
#include <random>
#include <span>
#include <vector>

#include <CDT.h>
#include <Splines.h>

#include "CdtOperators.h"
#include "Random.h"
#include "SplineOperators.h"

namespace smoothie
{

/// <summary>
/// Accessor to x field if input vector type does not have public x field.
/// </summary>
/// <typeparam name="T">Vector type, e.g glm::vec2</typeparam>
/// <param name="v">Vector variable</param>
/// <returns>x value of this vector</returns>
template <typename T>
float getX(const T &v);

/// <summary>
/// Accessor to y field if input vector type does not have public y field.
/// </summary>
/// <typeparam name="T">Vector type, e.g glm::vec2</typeparam>
/// <param name="v">Vector variable</param>
/// <returns>y value of this vector</returns>
template <typename T>
float getY(const T &v);

/// <summary>
/// Represents smoothed polygons.
/// Template parameter should either have public x and y fields
/// or specializations of getX&lt;Vec&gt; and getY&lt;Vec&gt; must be defined.
/// It must also be constructible with two parameters:
/// <code>
/// Vec x{ 1.0f, 2.0f };
/// </code>
/// </summary>
template <typename Vec>
class Polygon
{
public:
    /// <summary>
    /// Constructs smoothed polygon
    /// </summary>
    /// <param name="first">Iterator pointing to first contour point</param>
    /// <param name="last">Iterator after last contour point</param>
    /// <param name="tension">Splines tension, see Splines lib docs for details</param>
    /// <param name="segmentLength">Length of splines interpolation segments</param>
    template <typename InputIt>
    Polygon(InputIt first, InputIt last, float tension, float segmentLength);

    /// <summary>
    /// Outputs triangles vertices and indices to given output iterators
    /// </summary>
    /// <typeparam name="VerticesIt">Vertices output iterator type</typeparam>
    /// <typeparam name="IndicesIt">Indices output iterator type</typeparam>
    /// <param name="vertices">Vertices output iterator</param>
    /// <param name="indices">Indices output iterator</param>
    template <typename VerticesIt, typename IndicesIt>
    void dumpTriangles(VerticesIt vertices, IndicesIt indices) const;

    /// <summary>
    /// Computes polygon's centroid
    /// </summary>
    /// <returns>Polygon's centroid</returns>
    Vec computeCentroid() const;

    /// <summary>
    /// Spawns random point uniformly across polygon
    /// </summary>
    /// <param name="random01">
    /// Random floats generator.
    /// Returned numbers must be within [0.0f..1.0f) range.
    /// </param>
    /// <returns>Random point within polygon</returns>
    template <typename T = decltype(Random::random01)>
    Vec spawnRandomPoint(T random01 = Random::random01) const;

    /// <summary>
    /// Computes closest point on polygon's contour
    /// </summary>
    /// <param name="v">Target point</param>
    /// <returns>Point on the contour closest to v</returns>
    Vec computeClosestPoint(const Vec &v) const;

    /// <summary>
    /// Polygon's total square
    /// </summary>
    float square() const;

    /// <summary>
    /// Polygon's contour
    /// </summary>
    std::span<Vec> contour() const { return contour_; }

private:
    template <typename T>
    const CDT::Triangle *getRandomTriangle(T random01) const;

    std::vector<SplineLib::cSpline2> splines_;
    std::vector<Vec> contour_;
    CDT::Triangulation<float> cdt_;
    std::map<float, const CDT::Triangle *> triangles_;
};

// Implemenation

template <typename T>
float getX(const T &v)
{
    return v.x;
}

template <typename T>
float getY(const T &v)
{
    return v.y;
}

namespace details
{

inline SplineLib::cSpline2 createBezierSpline(const SplineLib::Vec2f &a, const SplineLib::Vec2f &b,
    const SplineLib::Vec2f &c, const SplineLib::Vec2f &d, float tension = 0.0f)
{
    const auto s = (1.0f - tension) * (1.0f / 6.0f);
    const auto pb1 = b + s * (c - a);
    const auto pb2 = c - s * (d - b);
    return BezierSpline(b, pb1, pb2, c);
}

template <typename InputIt, typename Out>
void createContourSplines(InputIt first, InputIt last, float tension, Out out)
{
    const auto next = [&first, &last](auto it) {
        ++it;
        if (it == last) {
            it = first;
        }
        return it;
    };

    for (auto a = first, b = next(a), c = next(b), d = next(c); a != last; ++a) {
        *out = createBezierSpline(*a, *b, *c, *d, tension);
        ++out;

        b = c;
        c = d;
        d = next(d);
    }
}

template <typename InputIt>
std::vector<SplineLib::cSpline2> createContourSplines(InputIt first, InputIt last, float tension)
{
    const auto size = std::distance(first, last);
    if (size < 2) {
        return {};
    }

    std::vector<SplineLib::cSpline2> result;
    result.reserve(size);
    createContourSplines(first, last, tension, std::back_inserter(result));
    return result;
}

template <typename Out>
void approximateSpline(const SplineLib::cSpline2 &spline, const float segmentLength, Out out)
{
    const auto length = SplineLib::Length(spline);
    const auto n = static_cast<int>(length / segmentLength) + 1;
    const auto nf = static_cast<float>(n);

    for (int i = 1; i < n; i++) {
        const auto t = static_cast<float>(i) / nf;
        const auto p = SplineLib::Position(spline, t);
        *out = { p.x, p.y };
        ++out;
    }

    const auto p = SplineLib::Position1(spline);
    *out = { p.x, p.y };
    ++out;
}

template <typename Vec>
std::vector<Vec> approximateContour(std::vector<SplineLib::cSpline2> splines,
    const float segmentLength)
{
    std::vector<Vec> result;
    for (const auto &spline : splines) {
        approximateSpline(spline, segmentLength, std::back_inserter(result));
    }
    return result;
}

template <typename It>
CDT::Triangulation<float> triangulate(It first, It last)
{
    CDT::Triangulation<float> cdt;

    using Vec = std::decay_t<decltype(*first)>;
    cdt.insertVertices(first, last, getX<Vec>, getY<Vec>);

    class IntIterator
    {
    public:
        explicit IntIterator(int i) : i{ i } {}

        int operator*() const
        {
            return i;
        }

        bool operator==(const IntIterator &rhs) const
        {
            return i == rhs.i;
        }

        IntIterator &operator++()
        {
            i += 1;
            return *this;
        }

        IntIterator operator++(int)
        {
            IntIterator result{ *this };
            i += 1;
            return result;
        }

        IntIterator operator+(const int v) const
        {
            return IntIterator{ i + v };
        }

    private:
        int i;
    };

    const auto size = static_cast<int>(std::distance(first, last));
    cdt.insertEdges(IntIterator{ 0 }, IntIterator{ size },
        [](const auto i) { return i; },
        [size](const auto i) { return (i + 1) % size; });

    cdt.eraseOuterTrianglesAndHoles();

    return cdt;
}

inline auto createTrianglesMap(const CDT::Triangulation<float> &cdt)
{
    const auto &vertices = cdt.vertices;

    std::map<float, const CDT::Triangle *> result;

    float total{ 0.0f };
    for (const auto &t : cdt.triangles) {
        const auto &a = vertices[t.vertices[0]];
        const auto &b = vertices[t.vertices[1]];
        const auto &c = vertices[t.vertices[2]];

        const auto u = b - a;
        const auto v = c - a;
        total += u.x * v.y - u.y * v.x;
        result.emplace(total * 0.5f, &t);
    }

    return result;
}

template <typename A, typename B>
A add(const A &a, const B &b)
{
    return { getX(a) + getX(b), getY(a) + getY(b) };
}

template <typename A, typename B>
A sub(const A &a, const B &b)
{
    return { getX(a) - getX(b), getY(a) - getY(b) };
}

template <typename Vec>
Vec mul(const Vec &a, const float s)
{
    return { getX(a) * s, getY(a) * s };
}

}

template <typename Vec>
template <typename InputIt>
Polygon<Vec>::Polygon(InputIt first, InputIt last, float tension, const float segmentLength)
    : splines_{ details::createContourSplines(first, last, tension) }
    , contour_{ details::approximateContour<Vec>(splines_, segmentLength) }
    , cdt_{ details::triangulate(contour_.begin(), contour_.end()) }
    , triangles_{ details::createTrianglesMap(cdt_) }
{
}

template <typename Vec>
Vec Polygon<Vec>::computeClosestPoint(const Vec &v) const
{
    Vec result{ 0.0f, 0.0f };
    if (splines_.empty()) {
        return result;
    }

    float minDistanceSq = std::numeric_limits<float>::max();

    const SplineLib::Vec2f vv{ getX(v), getY(v) };
    for (const auto &s : splines_) {
        const auto t = SplineLib::FindClosestPoint(vv, s);
        const auto p = SplineLib::Position(s, t);
        const auto delta = details::sub(p, v);
        const auto distanceSq = getX(delta) * getX(delta) + getY(delta) * getY(delta);
        if (minDistanceSq > distanceSq) {
            minDistanceSq = distanceSq;
            result = { p.x, p.y };
        }
    }

    return result;
}

template <typename Vec>
float Polygon<Vec>::square() const
{
    return triangles_.empty() ? 0.0f : triangles_.rbegin()->first;
}

template <typename Vec>
Vec Polygon<Vec>::computeCentroid() const
{
    Vec c{ 0.0f, 0.0f };

    const auto divisor = 6.0f * square();
    for (size_t i = 0; i < contour_.size(); i++) {
        const auto &a = contour_[i];
        const auto &b = contour_[(i + 1) % contour_.size()];

        const auto cp = getX(a) * getY(b) - getY(a) * getX(b);
        c = details::add(c, details::mul(details::add(a, b), cp / divisor));
    }

    return c;
}

template <typename Vec>
template <typename T>
const CDT::Triangle *Polygon<Vec>::getRandomTriangle(T random01) const
{
    const auto it = triangles_.lower_bound(random01() * square());
    if (it != triangles_.end()) {
        return it->second;
    }

    return triangles_.rbegin()->second;
}

template <typename Vec>
template <typename T>
Vec Polygon<Vec>::spawnRandomPoint(T random01) const
{
    const auto t = getRandomTriangle(random01);

    const auto &vertices = cdt_.vertices;
    const auto &a = vertices[t->vertices[0]];
    const auto &b = vertices[t->vertices[1]];
    const auto &c = vertices[t->vertices[2]];

    const auto r1 = random01();
    const auto r2 = random01();

    const auto sq1 = sqrt(r1);
    const auto v = a * (1.0f - sq1) + b * (sq1 * (1.0f - r2)) + c * (r2 * sq1);
    return { v.x, v.y };
}

template <typename Vec>
template <typename VerticesIt, typename IndicesIt>
void Polygon<Vec>::dumpTriangles(VerticesIt vertices, IndicesIt indices) const
{
    for (const auto &v : cdt_.vertices) {
        *vertices = { v.x, v.y };
        ++vertices;
    }

    for (const auto &t : cdt_.triangles) {
        for (int i = 0; i < 3; i++) {
            *indices = t.vertices[i];
            ++indices;
        }
    }
}

}
