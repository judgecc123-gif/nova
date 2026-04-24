#pragma once
#include <cmath>
#include <format>
#include <random>

namespace nova
{
template <typename T>
concept StandardVector2 = requires(T a) {
    a.x;
    a.y;
    requires std::same_as<decltype(a.x), decltype(a.y)>;
    requires sizeof(T) == 2 * sizeof(decltype(a.x));
};

template <typename T>
struct Vector2
{
    using value_type = T;
    T x{}, y{};
    auto operator<=>(const Vector2 &) const noexcept = default;

    // unary
    constexpr Vector2 operator+() const noexcept { return *this; }
    constexpr Vector2 operator-() const noexcept { return {-x, -y}; }

    // vector - vector
    constexpr Vector2 operator+(const Vector2 &o) const noexcept { return {x + o.x, y + o.y}; }
    constexpr Vector2 operator-(const Vector2 &o) const noexcept { return {x - o.x, y - o.y}; }
    constexpr Vector2 operator*(const Vector2 &o) const noexcept { return {x * o.x, y * o.y}; }
    constexpr Vector2 operator/(const Vector2 &o) const noexcept { return {x / o.x, y / o.y}; }

    // vector - scalar
    constexpr Vector2 operator*(T s) const noexcept { return {x * s, y * s}; }
    constexpr Vector2 operator/(T s) const noexcept { return {x / s, y / s}; }

    // compound assignments
    constexpr Vector2 &operator+=(const Vector2 &o) noexcept
    {
        x += o.x;
        y += o.y;
        return *this;
    }
    constexpr Vector2 &operator-=(const Vector2 &o) noexcept
    {
        x -= o.x;
        y -= o.y;
        return *this;
    }
    constexpr Vector2 &operator*=(const Vector2 &o) noexcept
    {
        x *= o.x;
        y *= o.y;
        return *this;
    }
    constexpr Vector2 &operator/=(const Vector2 &o) noexcept
    {
        x /= o.x;
        y /= o.y;
        return *this;
    }

    constexpr Vector2 &operator*=(T s) noexcept
    {
        x *= s;
        y *= s;
        return *this;
    }
    constexpr Vector2 &operator/=(T s) noexcept
    {
        x /= s;
        y /= s;
        return *this;
    }

    // symmetric scalar multiplication
    friend constexpr Vector2 operator*(T s, const Vector2 &v) noexcept { return v * s; }

    template <StandardVector2 V>
    operator V() const noexcept
    {
        return V{static_cast<decltype(V{}.x)>(x), static_cast<decltype(V{}.y)>(y)};
    }
};

template <typename T>
struct Vector3
{
    using value_type = T;
    T x{}, y{}, z{};
    auto operator<=>(const Vector3 &) const noexcept = default;

    constexpr Vector3() noexcept = default;
    constexpr Vector3(T x_, T y_, T z_) noexcept : x(x_), y(y_), z(z_) {}

    // unary
    constexpr Vector3 operator+() const noexcept { return *this; }
    constexpr Vector3 operator-() const noexcept { return {-x, -y, -z}; }

    // vector - vector
    constexpr Vector3 operator+(const Vector3 &o) const noexcept
    {
        return {x + o.x, y + o.y, z + o.z};
    }
    constexpr Vector3 operator-(const Vector3 &o) const noexcept
    {
        return {x - o.x, y - o.y, z - o.z};
    }
    constexpr Vector3 operator*(const Vector3 &o) const noexcept
    {
        return {x * o.x, y * o.y, z * o.z};
    }
    constexpr Vector3 operator/(const Vector3 &o) const noexcept
    {
        return {x / o.x, y / o.y, z / o.z};
    }

    // vector - scalar
    constexpr Vector3 operator*(T s) const noexcept { return {x * s, y * s, z * s}; }
    constexpr Vector3 operator/(T s) const noexcept { return {x / s, y / s, z / s}; }

    // compound assignments
    constexpr Vector3 &operator+=(const Vector3 &o) noexcept
    {
        x += o.x;
        y += o.y;
        z += o.z;
        return *this;
    }
    constexpr Vector3 &operator-=(const Vector3 &o) noexcept
    {
        x -= o.x;
        y -= o.y;
        z -= o.z;
        return *this;
    }
    constexpr Vector3 &operator*=(const Vector3 &o) noexcept
    {
        x *= o.x;
        y *= o.y;
        z *= o.z;
        return *this;
    }
    constexpr Vector3 &operator/=(const Vector3 &o) noexcept
    {
        x /= o.x;
        y /= o.y;
        z /= o.z;
        return *this;
    }

    constexpr Vector3 &operator*=(T s) noexcept
    {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }
    constexpr Vector3 &operator/=(T s) noexcept
    {
        x /= s;
        y /= s;
        z /= s;
        return *this;
    }

    // symmetric scalar multiplication
    friend constexpr Vector3 operator*(T s, const Vector3 &v) noexcept { return v * s; }
};

namespace math
{

// dot product
template <typename T>
constexpr auto dot(const Vector2<T> &a, const Vector2<T> &b) noexcept
    -> decltype(a.x * b.x + a.y * b.y)
{
    return a.x * b.x + a.y * b.y;
}

template <typename T>
constexpr auto dot(const Vector3<T> &a, const Vector3<T> &b) noexcept
    -> decltype(a.x * b.x + a.y * b.y + a.z * b.z)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

template <typename T>
constexpr auto cross(const Vector3<T> &a, const Vector3<T> &b) noexcept
    -> Vector3<decltype(a.x * b.x)>
{
    using R = decltype(a.x * b.x);
    return Vector3<R>{static_cast<R>(a.y * b.z - a.z * b.y), static_cast<R>(a.z * b.x - a.x * b.z),
                      static_cast<R>(a.x * b.y - a.y * b.x)};
}

template <typename T>
constexpr auto cross(const Vector2<T> &a, const Vector2<T> &b) noexcept
    -> decltype(a.x * b.y - a.y * b.x)
{
    return a.x * b.y - a.y * b.x;
}

// squared length
template <typename T>
constexpr auto length_squared(const Vector2<T> &v) noexcept -> decltype(v.x * v.x + v.y * v.y)
{
    return v.x * v.x + v.y * v.y;
}

template <typename T>
constexpr auto length_squared(const Vector3<T> &v) noexcept
    -> decltype(v.x * v.x + v.y * v.y + v.z * v.z)
{
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

template <typename T>
constexpr auto cos(const Vector2<T> &a, const Vector2<T> &b)
{
    return dot(a, b) / std::sqrt(length_squared(a) * length_squared(b));
}

template <typename T>
auto length(const Vector2<T> &v)
{
    return std::sqrt(v.x * v.x + v.y * v.y);
}

template <typename T>
auto length(const Vector3<T> &v)
{
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

// distance and squared distance
template <typename T>
constexpr auto distance_squared(const Vector2<T> &a, const Vector2<T> &b) noexcept
    -> decltype(length_squared(a - b))
{
    return length_squared(a - b);
}

template <typename T>
constexpr auto distance_squared(const Vector3<T> &a, const Vector3<T> &b) noexcept
    -> decltype(length_squared(a - b))
{
    return length_squared(a - b);
}

template <typename T>
constexpr auto distance(const Vector2<T> &a, const Vector2<T> &b)
{
    return std::sqrt(distance_squared(a, b));
}

template <typename T>
constexpr auto distance(const Vector3<T> &a, const Vector3<T> &b)
{
    return std::sqrt(distance_squared(a, b));
}

// normalize (returns zero/unchanged when length is zero)
template <typename T>
constexpr auto normalize(const Vector2<T> &v)
{
    auto len = length(v);
    return len == 0 ? v : v / len;
}

template <typename T>
constexpr auto normalize(const Vector3<T> &v)
{
    auto len = length(v);
    return len == 0 ? v : v / len;
}

// linear interpolation
template <typename T, typename S>
constexpr auto lerp(const Vector2<T> &a, const Vector2<T> &b, S t)
{
    return a + (b - a) * static_cast<T>(t);
}

template <typename T, typename S>
constexpr auto lerp(const Vector3<T> &a, const Vector3<T> &b, S t)
{
    return a + (b - a) * static_cast<T>(t);
}

// projection of a onto b
template <typename T>
constexpr auto project(const Vector2<T> &a, const Vector2<T> &b)
{
    auto denom = dot(b, b);
    if (denom == 0) return Vector2<T>{};
    return b * (dot(a, b) / denom);
}

template <typename T>
constexpr auto project(const Vector3<T> &a, const Vector3<T> &b)
{
    auto denom = dot(b, b);
    if (denom == 0) return Vector3<T>{};
    return b * (dot(a, b) / denom);
}

template <typename T>
constexpr auto reject(const Vector2<T> &a, const Vector2<T> &b)
{
    return a - project(a, b);
}

template <typename T>
constexpr T project_length(const Vector2<T> &a, const Vector2<T> &b)
{
    auto denom = dot(b, b);
    if (denom == 0) return T{};
    return dot(a, b) / sqrt(denom);
}

// reflect v around normal n (n need not be normalized)
template <typename T>
auto reflect(const Vector2<T> &v, const Vector2<T> &n)
{
    auto nn = dot(n, n);
    if (nn == 0) return v;
    return v - n * (static_cast<T>(2) * dot(v, n) / nn);
}

template <typename T>
auto reflect(const Vector3<T> &v, const Vector3<T> &n)
{
    auto nn = dot(n, n);
    if (nn == 0) return v;
    return v - n * (static_cast<T>(2) * dot(v, n) / nn);
}

}  // namespace math

using Vector2i = Vector2<int>;
using Vector2f = Vector2<float>;
using Vector2d = Vector2<double>;
using Vector3i = Vector3<int>;
using Vector3f = Vector3<float>;
using Vector3d = Vector3<double>;

template <typename T>
struct Matrix3x2
{
    T m00{}, m01{};
    T m10{}, m11{};
    T m20{}, m21{};

    constexpr auto operator<=>(const Matrix3x2 &) const noexcept = default;

    constexpr T &operator[](size_t row, size_t col)
    {
        switch (row * 2 + col) {
            case 0:
                return m00;
            case 1:
                return m01;
            case 2:
                return m10;
            case 3:
                return m11;
            case 4:
                return m20;
            case 5:
                return m21;
            default:
                throw std::out_of_range("Matrix3x2 index out of range");
        }
    }

    constexpr const T &operator[](size_t row, size_t col) const
    {
        switch (row * 2 + col) {
            case 0:
                return m00;
            case 1:
                return m01;
            case 2:
                return m10;
            case 3:
                return m11;
            case 4:
                return m20;
            case 5:
                return m21;
            default:
                throw std::out_of_range("Matrix3x2 index out of range");
        }
    }
};

namespace math
{
template <typename T>
constexpr Vector2<T> transform(const Vector2<T> &v, const Matrix3x2<T> &m) noexcept
{
    return {
        v.x * m.m00 + v.y * m.m10 + m.m20,
        v.x * m.m01 + v.y * m.m11 + m.m21,
    };
}

template <typename T>
constexpr Matrix3x2<T> identity() noexcept
{
    Matrix3x2<T> result;
    result.m00 = 1;
    result.m11 = 1;
    return result;
}

template <typename T>
constexpr Vector2<T> inverse(const Matrix3x2<T> &m)
{
    T det = m.m00 * m.m11 - m.m10 * m.m01;
    if (det == 0) throw std::runtime_error("Matrix is not invertible");

    Matrix3x2<T> inv;
    inv.m00 = m.m11 / det;
    inv.m01 = -m.m01 / det;
    inv.m10 = -m.m10 / det;
    inv.m11 = m.m00 / det;
    inv.m20 = (m.m10 * m.m21 - m.m20 * m.m11) / det;
    inv.m21 = (m.m20 * m.m01 - m.m00 * m.m21) / det;

    return inv;
}
}  // namespace math

using Matrix3x2f = Matrix3x2<float>;
using Matrix3x2d = Matrix3x2<double>;

template <class T>
class Singleton
{
protected:
    Singleton() = default;

private:
    Singleton(const Singleton &)            = delete;
    Singleton &operator=(const Singleton &) = delete;

public:
    static T &instance() noexcept(std::is_nothrow_default_constructible_v<T>)
    {
        static T ins;
        return ins;
    }
};

class Random final : public Singleton<Random>
{
    friend class Singleton<Random>;

    Random() noexcept = default;

    std::mt19937 engine{std::random_device{}()};

public:
    auto operator()(const auto &distribution) { return distribution(engine); }

    bool operator()(double possibility) { return std::bernoulli_distribution(possibility)(engine); }

    // [min,max]
    template <std::integral Int>
    Int operator()(Int min, Int max)
    {
        return std::uniform_int_distribution<Int>{min, max}(engine);
    }

    // [min,max)
    template <std::floating_point R>
    R operator()(R min, R max)
    {
        return std::uniform_real_distribution<R>{min, max}(engine);
    }

    template <class T>
    nova::Vector2<T> operator()(const nova::Vector2<T> &min, const nova::Vector2<T> &max)
    {
        return {(*this)(min.x, max.x), (*this)(min.y, max.y)};
    }
    template <class T>
    nova::Vector3<T> operator()(const nova::Vector3<T> &min, const nova::Vector3<T> &max)
    {
        return {(*this)(min.x, max.x), (*this)(min.y, max.y), (*this)(min.z, max.z)};
    }
};

}  // namespace nova

namespace std
{

template <typename T>
class formatter<nova::Vector2<T>, char> : public std::formatter<std::string, char>
{
public:
    formatter() = default;

    constexpr auto format(nova::Vector2<T> lhs, auto &ctx) const
    {
        return std::formatter<std::string, char>::format(std::format("({},{})", lhs.x, lhs.y), ctx);
    }
};

template <typename T>
class formatter<nova::Vector3<T>, char> : public std::formatter<std::string, char>
{
public:
    formatter() = default;

    constexpr auto format(nova::Vector3<T> lhs, auto &ctx) const
    {
        return std::formatter<std::string, char>::format(
            std::format("({},{},{})", lhs.x, lhs.y, lhs.z), ctx);
    }
};

template <typename T>
class formatter<nova::Matrix3x2<T>, char> : public std::formatter<std::string, char>
{
public:
    formatter() = default;

    constexpr auto format(nova::Matrix3x2<T> lhs, auto &ctx) const
    {
        return std::formatter<std::string, char>::format(
            std::format("(({},{}),({},{}),({},{}))", lhs.m00, lhs.m01, lhs.m10, lhs.m11, lhs.m20,
                        lhs.m21),
            ctx);
    }
};

}  // namespace std
