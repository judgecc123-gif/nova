#pragma once

#include <d2d1_3.h>

#include <cmath>
#include <common.hpp>

namespace nova
{
namespace rnd
{

class View2D
{
public:
    Vector2f center{};
    float zoom{1.0};
    float rotation{0.0};
    Vector2<bool> flip{false, false};

    Vector2f WorldToScreen(Vector2f worldPos, Vector2f screen_size) const
    {
        auto offset = worldPos - center;
        offset      = offset * zoom;

        float cosR = std::cos(rotation);
        float sinR = std::sin(rotation);
        Vector2f rotated{offset.x * cosR - offset.y * sinR, offset.x * sinR + offset.y * cosR};

        if (flip.x) {
            rotated.x = -rotated.x;
        }
        if (flip.y) {
            rotated.y = -rotated.y;
        }

        return {rotated.x + screen_size.x * 0.5f, rotated.y + screen_size.y * 0.5f};
    }

    Vector2f ScreenToWorld(Vector2f screenPos, Vector2f screen_size) const
    {
        Vector2f centered{screenPos.x - screen_size.x * 0.5f, screenPos.y - screen_size.y * 0.5f};

        if (flip.x) {
            centered.x = -centered.x;
        }
        if (flip.y) {
            centered.y = -centered.y;
        }

        float cosR = std::cos(-rotation);
        float sinR = std::sin(-rotation);
        Vector2f unrotated{centered.x * cosR - centered.y * sinR,
                           centered.x * sinR + centered.y * cosR};

        unrotated = unrotated / zoom;
        return unrotated + center;
    }

    Matrix3x2f GetTransformMatrix(Vector2f screen_size) const
    {
        float cosR = std::cos(rotation);
        float sinR = std::sin(rotation);

        float sx = static_cast<float>(zoom * cosR);
        float sy = static_cast<float>(zoom * sinR);
        float tx = static_cast<float>(-center.x * sx + center.y * sy + screen_size.x * 0.5);
        float ty = static_cast<float>(-center.x * sy - center.y * sx + screen_size.y * 0.5);

        if (flip.y) {
            sy = -sy;
            ty = -ty + screen_size.y;
        }
        if (flip.x) {
            sx = -sx;
            tx = -tx + screen_size.x;
        }

        return {sx, sy, -sy, sx, tx, ty};
    }
};

}  // namespace rnd
}  // namespace nova