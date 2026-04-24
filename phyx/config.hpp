#include <bit>

#include "common.hpp"

namespace nova::phy
{
using Real                    = double;
inline constexpr Real epsilon = static_cast<Real>(1e-7);
inline constexpr Real pi      = static_cast<Real>(3.14159265358979323846);

using Vector2r = nova::Vector2<Real>;
using Vector3r = nova::Vector3<Real>;
}  // namespace nova::phy
