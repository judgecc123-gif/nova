#include "gravity.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>
#include <utility>
#include <vector>

namespace nova::phy
{

bool GravityWorld::collideAABBs(size_t i, size_t j) const
{
    return !(shape[i].right < shape[j].left || shape[i].left > shape[j].right ||
             shape[i].bottom < shape[j].top || shape[i].top > shape[j].bottom);
}

auto GravityWorld::collideCircles(Vector2r p1, Real r1, Vector2r p2, Real r2) const
    -> std::pair<bool, CollisionResult>
{
    Vector2r delta  = p2 - p1;
    Real dist_sq    = math::length_squared(delta);
    Real radius_sum = r1 + r2;

    if (dist_sq >= radius_sum * radius_sum) return std::make_pair(false, CollisionResult{});

    Real dist       = std::sqrt(dist_sq);
    Vector2r normal = (dist > epsilon) ? delta / dist : Vector2r{0, 1};
    Real depth      = radius_sum - dist;

    Vector2r worldContact = p1 + normal * (r1 - depth * 0.5);

    return std::make_pair(
        true, CollisionResult{
                  .normal = normal, .contact = {worldContact}, .contact_count = 1, .depth = depth});
}

auto GravityWorld::Clip(const std::vector<Vector2r> &vertices, Vector2r normal, Vector2r ref1,
                        Vector2r ref2) -> std::vector<Vector2r>
{
    size_t best_index = 0;
    Real min_dot      = math::dot(vertices[0], normal);
    for (auto i = 1uz; i < vertices.size(); ++i) {
        Real dot = math::dot(vertices[i], normal);
        if (min_dot > dot) {
            min_dot    = dot;
            best_index = i;
        }
    }

    auto pre     = (best_index + vertices.size() - 1) % vertices.size();
    auto next    = (best_index + 1) % vertices.size();
    auto ref_vec = ref2 - ref1;
    auto ref_dir = math::normalize(ref_vec);

    std::vector<Vector2r> points;
    if (std::abs(math::dot(ref_dir, math::normalize(vertices[best_index] - vertices[pre]))) >
        std::abs(math::dot(ref_dir, math::normalize(vertices[best_index] - vertices[next])))) {
        points = {vertices[pre], vertices[best_index]};
    } else {
        points = {vertices[best_index], vertices[next]};
    }

    auto clip_plane = [](const std::vector<Vector2r> &in_pts, Vector2r plane_normal,
                         Vector2r plane_point) {
        std::vector<Vector2r> out_pts;
        if (in_pts.size() < 2) return in_pts;

        for (size_t i = 0; i < in_pts.size(); ++i) {
            Vector2r p1 = in_pts[i];
            Vector2r p2 = in_pts[(i + 1) % in_pts.size()];

            Real d1 = math::dot(plane_normal, p1 - plane_point);
            Real d2 = math::dot(plane_normal, p2 - plane_point);

            if (d1 <= 0) out_pts.emplace_back(p1);
            if ((d1 <= 0 && d2 > 0) || (d1 > 0 && d2 <= 0)) {
                Real t = d1 / (d1 - d2);
                out_pts.emplace_back(p1 + t * (p2 - p1));
            }
        }
        return out_pts;
    };

    points = clip_plane(points, -ref_dir, ref1);
    points = clip_plane(points, ref_dir, ref2);

    return points | std::views::filter([&](auto p) { return math::dot(normal, p - ref1) <= 0; }) |
           std::ranges::to<std::vector<Vector2r>>();
}

auto GravityWorld::collideConvexes(Vector2r p1, const std::vector<Vector2r> &verts1, Vector2r p2,
                                   const std::vector<Vector2r> &verts2) const
    -> std::pair<bool, CollisionResult>
{
    const auto overlapDot = [&](Vector2r axis) -> Real {
        Real min_proj1 = std::numeric_limits<Real>::max(),
             max_proj1 = std::numeric_limits<Real>::lowest();
        for (const auto &p : verts1) {
            Real proj = math::dot(p, axis);
            min_proj1 = std::min(min_proj1, proj);
            max_proj1 = std::max(max_proj1, proj);
        }
        Real min_proj2 = std::numeric_limits<Real>::max(),
             max_proj2 = std::numeric_limits<Real>::lowest();
        for (const auto &p : verts2) {
            Real proj = math::dot(p, axis);
            min_proj2 = std::min(min_proj2, proj);
            max_proj2 = std::max(max_proj2, proj);
        }
        if (max_proj1 < min_proj2 || max_proj2 < min_proj1)
            return std::numeric_limits<Real>::quiet_NaN();
        return std::min(max_proj1, max_proj2) - std::max(min_proj1, min_proj2);
    };

    Real min_overlap = std::numeric_limits<Real>::max();
    Vector2r normal;

    int ref_index         = -1;
    size_t ref_edge_index = 0;

    for (auto i = 0uz; i < verts1.size(); ++i) {
        Vector2r edge = verts1[(i + 1) % verts1.size()] - verts1[i];
        Vector2r axis{edge.y, -edge.x};

        Real overlap_dot = overlapDot(axis);
        if (std::isnan(overlap_dot)) return std::make_pair(false, CollisionResult{});

        Real axis_length = math::length(axis);
        Real overlap     = overlap_dot / axis_length;
        if (overlap < epsilon) return std::make_pair(false, CollisionResult{});
        if (overlap < min_overlap) {
            min_overlap    = overlap;
            normal         = axis / axis_length;
            ref_index      = 0;
            ref_edge_index = i;
        }
    }

    for (auto i = 0uz; i < verts2.size(); ++i) {
        Vector2r edge = verts2[(i + 1) % verts2.size()] - verts2[i];
        Vector2r axis{edge.y, -edge.x};

        Real overlap_dot = overlapDot(axis);
        if (std::isnan(overlap_dot)) return std::make_pair(false, CollisionResult{});

        Real axis_length = math::length(axis);
        Real overlap     = overlap_dot / axis_length;
        if (overlap < epsilon) return std::make_pair(false, CollisionResult{});
        if (overlap < min_overlap) {
            min_overlap    = overlap;
            normal         = axis / axis_length;
            ref_index      = 1;
            ref_edge_index = i;
        }
    }

    std::array<Vector2r, 2> contacts;
    uint8_t count{};
    if (ref_index == 0) {
        auto v = Clip(verts2, normal, verts1[ref_edge_index],
                      verts1[(ref_edge_index + 1) % verts1.size()]);
        for (int i = 0; i < v.size(); ++i) contacts[i] = v[i];
        count = static_cast<uint8_t>(v.size());
    } else {
        auto v = Clip(verts1, -normal, verts2[ref_edge_index],
                      verts2[(ref_edge_index + 1) % verts2.size()]);
        for (int i = 0; i < v.size(); ++i) contacts[i] = v[i];
        count = static_cast<uint8_t>(v.size());
    }

    if (math::dot(p2 - p1, normal) < 0) normal = -normal;

    return std::make_pair(
        true,
        CollisionResult{
            .normal = normal, .contact = contacts, .contact_count = count, .depth = min_overlap});
}

auto GravityWorld::collideCircleConvex(Vector2r circlePos, Real radius, Vector2r convPos,
                                       const std::vector<Vector2r> &verts) const
    -> std::pair<bool, CollisionResult>
{
    const auto solveOverlap = [&](Vector2r axis) -> Real {
        Real min_proj1 = std::numeric_limits<Real>::max(),
             max_proj1 = std::numeric_limits<Real>::lowest();
        Real inv_len   = 1.0 / math::length(axis);
        for (const auto &p : verts) {
            Real proj = math::dot(p, axis) * inv_len;
            min_proj1 = std::min(min_proj1, proj);
            max_proj1 = std::max(max_proj1, proj);
        }
        Real proj      = math::dot(circlePos, axis) * inv_len;
        auto min_proj2 = proj - radius;
        auto max_proj2 = proj + radius;
        if (max_proj1 < min_proj2 || max_proj2 < min_proj1)
            return std::numeric_limits<Real>::quiet_NaN();
        return std::min(max_proj1, max_proj2) - std::max(min_proj1, min_proj2);
    };

    Real min_overlap = std::numeric_limits<Real>::max();
    Vector2r normal;
    for (auto i = 0uz; i < verts.size(); ++i) {
        Vector2r edge = verts[(i + 1) % verts.size()] - verts[i];
        Vector2r axis{-edge.y, edge.x};

        Real overlap = solveOverlap(axis);
        if (std::isnan(overlap) || overlap < epsilon)
            return std::make_pair(false, CollisionResult{});

        if (overlap < min_overlap) {
            min_overlap = overlap;
            normal      = axis;
        }
    }

    Vector2r closest;
    Real min_dist_sq = std::numeric_limits<Real>::max();
    for (const auto &v : verts) {
        Real dist_sq = math::distance_squared(circlePos, v);
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            closest     = v;
        }
    }
    const auto axis = closest - circlePos;

    Real overlap = solveOverlap(axis);
    if (std::isnan(overlap) || overlap < epsilon) return std::make_pair(false, CollisionResult{});
    if (overlap < min_overlap) {
        min_overlap = overlap;
        normal      = axis;
    }

    if (math::dot(normal, axis) < 0) normal = -normal;

    Vector2r worldContact = circlePos + normal * (radius - min_overlap * 0.5);

    return std::make_pair(
        true,
        CollisionResult{
            .normal = normal, .contact = {worldContact}, .contact_count = 1, .depth = min_overlap});
}

auto GravityWorld::collide(size_t i, size_t j) const -> std::pair<bool, CollisionResult>
{
    if (const auto &c1 = std::get_if<Circle>(&shape[i].data); c1) {
        if (const auto &c2 = std::get_if<Circle>(&shape[j].data); c1 && c2) {
            return collideCircles(position[i], c1->radius, position[j], c2->radius);
        }
        if (const auto &conv2 = std::get_if<Convex>(&shape[j].data); c1 && conv2) {
            return collideCircleConvex(position[i], c1->radius, position[j], conv2->transformed());
        }
    }
    if (const auto &conv1 = std::get_if<Convex>(&shape[i].data); conv1) {
        if (const auto &c2 = std::get_if<Circle>(&shape[j].data); conv1 && c2) {
            auto [overlap, result] =
                collideCircleConvex(position[j], c2->radius, position[i], conv1->transformed());
            result.normal = -result.normal;
            return std::make_pair(overlap, result);
        }
        if (const auto &conv2 = std::get_if<Convex>(&shape[j].data); conv1 && conv2) {
            return collideConvexes(position[i], conv1->transformed(), position[j],
                                   conv2->transformed());
        }
    }
    return std::make_pair(false, CollisionResult{});
}

void GravityWorld::Step(Real dt)
{
    const int solver_iterations = 8;

    for (auto i = 0uz; i < mass.size(); ++i) {
        if (mass[i] <= 0) continue;

        velocity[i] += ((force[i] / mass[i]) + gravity) * dt;
        angular_velocity[i] += (torque[i] / inertia[i]) * dt;

        force[i]  = {};
        torque[i] = {};
    }

    for (auto i = 0uz; i < mass.size(); ++i) {
        shape[i].updateTransformed(position[i], angle[i]);
    }

    struct ContactManifold
    {
        size_t id_A, id_B;
        CollisionResult result;
    };
    std::vector<ContactManifold> manifolds;

    for (auto i = 0uz; i < mass.size(); ++i) {
        for (auto j = i + 1; j < mass.size(); ++j) {
            if (auto [collided, res] = collide(i, j); collided) {
                manifolds.emplace_back(i, j, res);
            }
        }
    }

    for (int iter = 0; iter < solver_iterations; ++iter) {
        for (auto &m : manifolds) {
            size_t i        = m.id_A;
            size_t j        = m.id_B;
            const auto &res = m.result;

            for (uint32_t k = 0; k < res.contact_count; ++k) {
                Vector2r world_point = res.contact[k];
                Vector2r ra          = world_point - position[i];
                Vector2r rb          = world_point - position[j];

                Vector2r va =
                    velocity[i] + Vector2r(-angular_velocity[i] * ra.y, angular_velocity[i] * ra.x);
                Vector2r vb =
                    velocity[j] + Vector2r(-angular_velocity[j] * rb.y, angular_velocity[j] * rb.x);
                Vector2r v_rel = vb - va;

                Real rel_vel_n = math::dot(v_rel, res.normal);
                if (rel_vel_n > 0) continue;

                Real ra_n = math::cross(ra, res.normal);
                Real rb_n = math::cross(rb, res.normal);
                Real K    = (1.0 / mass[i]) + (1.0 / mass[j]) + (ra_n * ra_n) / inertia[i] +
                         (rb_n * rb_n) / inertia[j];

                Real beta = 0.2;
                Real slop = 0.01;
                Real bias = (beta / dt) * std::max(0.0, res.depth - slop);

                Real impulse_mag = (-(1.0 + 0.0) * rel_vel_n + bias) / K;
                impulse_mag /= res.contact_count;

                Vector2r P = res.normal * impulse_mag;

                velocity[i] -= P / mass[i];
                angular_velocity[i] -= math::cross(ra, P) / inertia[i];

                velocity[j] += P / mass[j];
                angular_velocity[j] += math::cross(rb, P) / inertia[j];
            }
        }
    }

    for (auto i = 0uz; i < mass.size(); ++i) {
        position[i] += velocity[i] * dt;
        angle[i] += angular_velocity[i] * dt;
    }
}

}  // namespace nova::phy
