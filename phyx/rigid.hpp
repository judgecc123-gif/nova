#pragma once

#include <cassert>

#include "shape.hpp"

namespace nova::phy
{
struct RigidBody
{
    Vector2r position, velocity, force;
    Real mass{};
    Real angle{};
    Real angular_velocity{};
    Real torque{};
    Real density{};
};

template <typename WorldType>
concept HasMass = requires(WorldType w) {
    { w.mass } -> std::convertible_to<std::vector<Real>>;
};

template <typename WorldType>
concept HasForce = requires(WorldType w) {
    { w.force } -> std::convertible_to<std::vector<Vector2r>>;
};

template <typename WorldType>
concept HasShape = requires(WorldType w) {
    { w.shape } -> std::convertible_to<std::vector<Shape>>;
};

template <typename WorldType>
concept HasDensity = HasShape<WorldType> && HasMass<WorldType> && requires(WorldType w) {
    { w.density } -> std::convertible_to<std::vector<Real>>;
};

template <typename WorldType>
concept HasAngle = HasShape<WorldType> && requires(WorldType w) {
    { w.angle } -> std::convertible_to<std::vector<Real>>;
    { w.angular_velocity } -> std::convertible_to<std::vector<Real>>;
    { w.torque } -> std::convertible_to<std::vector<Real>>;
    { w.inertia } -> std::convertible_to<std::vector<Real>>;
};

template <typename WorldType>
class BodyToken
{
    friend WorldType;

private:
    size_t id{};
    WorldType *world{};

    BodyToken(size_t id_, WorldType *world_) : id(id_), world(world_) {}

public:
    BodyToken() = default;

    // 位置，速度
    Vector2r GetPosition() const
    {
        assert(world && id < world->position.size());
        return world->position[id];
    }
    void SetPosition(Vector2r p)
    {
        assert(world && id < world->position.size());
        world->position[id] = p;
    }
    Vector2r GetVelocity() const
    {
        assert(world && id < world->velocity.size());
        return world->velocity[id];
    }
    void SetVelocity(Vector2r v)
    {
        assert(world && id < world->velocity.size());
        world->velocity[id] = v;
    }

    // 质量和密度
    Real GetMass() const
    {
        assert(world && id < world->mass.size());
        return world->mass[id];
    }
    void SetMass(Real m)
    {
        assert(world && id < world->mass.size());
        world->mass[id] = m;
    }

    Real GetDensity() const

    {
        assert(world && id < world->density.size());
        return world->density[id];
    }
    void SetDensity(Real d)

    {
        assert(world && id < world->density.size());
        world->density[id] = d;
        world->mass[id]    = d * world->shape[id].area();

        if constexpr (HasAngle<WorldType>) {
            world->inertia[id] = d * world->shape[id].inertia();
        }
    }

    // 形状
    const Shape &GetShape() const

    {
        assert(world && id < world->shape.size());
        return world->shape[id];
    }

    template <typename ShapeType>
    void SetShape(ShapeType &&s)

    {
        assert(world && id < world->shape.size());
        world->shape[id] = std::forward<ShapeType>(s);
        world->mass[id]  = world->density[id] * s.area();

        if constexpr (HasAngle<WorldType>) {
            world->inertia[id] = world->density[id] * s.inertia();
        }
    }

    // 角度和角速度
    Real GetAngle() const
    {
        assert(world && id < world->angle.size());
        return world->angle[id];
    }
    Real GetAngularVelocity() const
    {
        assert(world && id < world->angular_velocity.size());
        return world->angular_velocity[id];
    }
    void SetAngularVelocity(Real w)
    {
        assert(world && id < world->angular_velocity.size());
        world->angular_velocity[id] = w;
    }

    // 力和力矩
    void ApplyForce(Vector2r f)
    {
        assert(world && id < world->force.size());
        world->force[id] += f;
    }
    void ApplyForce(Vector2r f, Vector2r contact)
    {
        assert(world && id < world->force.size() && id < world->position.size());
        world->force[id] += f;
        world->torque[id] += math::cross(contact - world->position[id], f);
    }

    void ApplyTorque(Real tau)
    {
        assert(world && id < world->torque.size());
        world->torque[id] += tau;
    }

    size_t GetId() const { return id; }
    bool IsValid() const { return world != nullptr && id < world->position.size(); }

    bool operator==(const BodyToken &) const = default;
    operator bool() const { return IsValid(); }
};

}  // namespace nova::phy
