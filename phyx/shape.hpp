#pragma once

#include <algorithm>
#include <print>
#include <variant>
#include <vector>

#include "config.hpp"

namespace nova
{
namespace phy
{

struct Circle
{
    Real radius;
    constexpr Circle(Real radius) noexcept : radius(radius) {}

    constexpr Real area() const noexcept { return pi * radius * radius; }
    constexpr Real inertia() const noexcept
    {
        return 0.5f * pi * radius * radius * radius * radius;
    }
};

struct Convex
{
    friend struct Shape;

    Convex(std::vector<Vector2r> verts) : vertices_(std::move(verts))
    {
        if (vertices_.size() < 3)
            throw std::invalid_argument("Convex shape must have at least 3 vertices.");

        Vector2r centroid{};

        for (auto i = 1uz; i < vertices_.size(); ++i) {
            vertices_[i] -= vertices_[0];
        }
        vertices_[0] = {};

        Real total{};
        for (auto i = 1uz; i < vertices_.size() - 1; ++i) {
            Real cross = math::cross(vertices_[i], vertices_[i + 1]);
            total += cross;
            centroid += (vertices_[i] + vertices_[i + 1]) * cross;
        }
        centroid /= total * 3;

        for (auto& v : vertices_) {
            v -= centroid;
        }

        if (total < 0) {
            std::println("vertices are in clockwise order, reversing to counter-clockwise.");
            std::ranges::reverse(vertices_);
            area_ = total * -0.5f;
        } else {
            area_ = total * 0.5f;
        }

        auto calc = [](Vector2r a, Vector2r b) static {
            return math::cross(a, b) *
                   (a.x * a.x + a.x * b.x + b.x * b.x + a.y * a.y + a.y * b.y + b.y * b.y);
        };

        inertia_ = {};
        for (auto i = 0uz; i < vertices_.size(); ++i) {
            inertia_ += calc(vertices_[i], vertices_[(i + 1) % vertices_.size()]);
        }
        inertia_ /= 12;
    }

    const std::vector<Vector2r>& vertices() const { return vertices_; }
    const std::vector<Vector2r>& transformed() const { return transformed_; }

    Real area() const { return area_; }

    Real inertia() const { return inertia_; }

private:
    std::vector<Vector2r> vertices_;
    std::vector<Vector2r> transformed_;
    Real area_;
    Real inertia_;
};

struct Shape
{
    ~Shape() {}

    std::variant<Circle, Convex> data;

    void updateTransformed(const Vector2r& position, Real angle)
    {
        if (auto convex = std::get_if<Convex>(&data); convex) {
            convex->transformed_.resize(convex->vertices_.size());
            for (size_t i = 0; i < convex->vertices_.size(); ++i) {
                auto x = convex->transformed_[i].x;
                auto y = convex->transformed_[i].y;

                Real sinA = std::sin(angle);
                Real cosA = std::cos(angle);

                convex->transformed_[i].x = x * cosA - y * sinA + position.x;
                convex->transformed_[i].y = x * sinA + y * cosA + position.y;
            }

            auto [l, r] =
                std::minmax_element(convex->transformed_.begin(), convex->transformed_.end(),
                                    [](const auto& a, const auto& b) { return a.x < b.x; });
            auto [t, b] =
                std::minmax_element(convex->transformed_.begin(), convex->transformed_.end(),
                                    [](const auto& a, const auto& b) { return a.y < b.y; });

            left   = l->x;
            right  = r->x;
            top    = t->y;
            bottom = b->y;
        } else if (auto circle = std::get_if<Circle>(&data); circle) {
            left   = position.x - circle->radius;
            right  = position.x + circle->radius;
            top    = position.y - circle->radius;
            bottom = position.y + circle->radius;
        }
    }

    // AABB
    Real left, right, top, bottom;

    Real area() const
    {
        return std::visit([](const auto& shape) { return shape.area(); }, data);
    }
    Real inertia() const
    {
        return std::visit([](const auto& shape) { return shape.inertia(); }, data);
    }

private:
};

}  // namespace phy
}  // namespace nova
