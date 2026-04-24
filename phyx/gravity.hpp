#include <array>
#include <ranges>
#include <utility>

#include "rigid.hpp"

namespace nova::phy
{
class GravityWorld
{
private:
    struct CollisionResult
    {
        Vector2r normal;  // from body i to j
        std::array<Vector2r, 2> contact;
        uint32_t contact_count;
        Real depth;
    };
    static constexpr auto x = (sizeof(CollisionResult));

    bool collideAABBs(size_t i, size_t j) const;

    auto collideCircles(Vector2r p1, Real r1, Vector2r p2, Real r2) const
        -> std::pair<bool, CollisionResult>;

    static auto Clip(const std::vector<Vector2r> &vertices, Vector2r normal, Vector2r ref1,
                     Vector2r ref2) -> std::vector<Vector2r>;

    //  vertices have been transformed, and p1, p2 only for determining normal
    //  direction
    auto collideConvexes(Vector2r p1, const std::vector<Vector2r> &verts1, Vector2r p2,
                         const std::vector<Vector2r> &verts2) const
        -> std::pair<bool, CollisionResult>;

    // vertices have been transformed, and pos only for determining normal
    // direction
    auto collideCircleConvex(Vector2r circlePos, Real radius, Vector2r convPos,
                             const std::vector<Vector2r> &verts) const
        -> std::pair<bool, CollisionResult>;

    std::vector<Real> mass, inertia, density;
    std::vector<Vector2r> position, velocity, force;
    std::vector<Real> angle, angular_velocity, torque;
    std::vector<Shape> shape;

public:
    struct BodyBuiltParam
    {
        Vector2r position, velocity;
        Real angle{};
        Real angular_velocity{};
        Real density{1.0};
    };

    Vector2r gravity;

    GravityWorld(Vector2r gravity = {}) : gravity(gravity) {}

    // 添加刚体到世界中
    template <typename ShapeType>
    auto CreateBody(const BodyBuiltParam &body, ShapeType &&shape_param)
    {
        density.emplace_back(body.density);
        position.emplace_back(body.position);
        velocity.emplace_back(body.velocity);
        angle.emplace_back(body.angle);
        angular_velocity.emplace_back(body.angular_velocity);

        force.emplace_back(Vector2r{});
        torque.emplace_back(Real{});

        shape.emplace_back(std::forward<ShapeType>(shape_param));

        mass.emplace_back(body.density * shape.back().area());
        inertia.emplace_back(body.density * shape.back().inertia());

        return BodyToken<GravityWorld>{mass.size() - 1, this};
    }

    // 获取刚体状态
    RigidBody GetBody(size_t i) const
    {
        if (i >= mass.size()) throw std::out_of_range("Body index out of range");
        return RigidBody{position[i], velocity[i],         force[i],  mass[i],
                         angle[i],    angular_velocity[i], torque[i], density[i]};
    }

    // 应用力和力矩（带接触点）
    void ApplyForce(size_t i, Vector2r f, Vector2r contact)
    {
        if (i < force.size()) {
            force[i] += f;
            torque[i] += math::cross(contact - position[i], f);
        }
    }

    size_t GetBodyCount() const { return mass.size(); }

    auto collide(size_t i, size_t j) const -> std::pair<bool, CollisionResult>;

    void Step(Real dt);
};
}  // namespace nova::phy