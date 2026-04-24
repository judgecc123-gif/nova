#include "rigid.hpp"

namespace nova::phy
{
class CelestialWorld
{
    friend class BodyToken<CelestialWorld>;

public:
    std::vector<Real> mass;
    std::vector<Vector2r> position, velocity, force;

    Real G;  // 万有引力常数 (单位：AU^3 / (yr^2 * solar_mass)，适用于天文单位)

    struct BodyBuiltParam
    {
        Vector2r position, velocity;
        Real mass;
    };

    auto CreateBody(const BodyBuiltParam &body)
    {
        position.emplace_back(body.position);
        velocity.emplace_back(body.velocity);
        mass.emplace_back(body.mass);
        force.emplace_back(Vector2r{});

        return BodyToken<CelestialWorld>{mass.size() - 1, this};
    }

    void Step(Real dt)
    {
        // 计算引力
        for (auto i = 0uz; i < mass.size(); ++i) {
            for (auto j = i + 1; j < mass.size(); ++j) {
                Vector2r dir = position[j] - position[i];
                Real dist_sq = math::length_squared(dir);
                if (dist_sq < 1e-10) continue;
                Real f_mag = G * mass[i] * mass[j] / dist_sq;

                dir = math::normalize(dir);
                force[i] += f_mag * dir;
                force[j] -= f_mag * dir;
            }
        }

        // 更新速度和位置
        for (auto i = 0uz; i < mass.size(); ++i) {
            velocity[i] += (force[i] / mass[i]) * dt;
            position[i] += velocity[i] * dt;
            force[i] = {};
        }
    }
};

}  // namespace nova::phy
