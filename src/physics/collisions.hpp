#ifndef _462_COLLISIONS_HPP_
#define _462_COLLISIONS_HPP_

#include "scene/sphere.hpp"
#include "physics/spherebody.hpp"
#include "physics/trianglebody.hpp"
#include "physics/planebody.hpp"

namespace _462
{

  bool collides(SphereBody &body1, SphereBody &body2, real_t collision_damping);
  bool collides(SphereBody &body1, TriangleBody &body2, real_t collision_damping);
  bool collides(SphereBody &body1, PlaneBody &body2, real_t collision_damping);

  // 判断两物体相对速度是否为正，即是否靠近
  real_t relative_velocity(SphereBody &body1, Vector3 velocity, Vector3 position);
  // 判断碰撞点是否落在三角形内
  bool is_in_triangle(const Vector3 point, const Vector3 vertices[3]);
  // 碰撞后减速
  Vector3 damping(Vector3 velocity, real_t collision_damping);

} // namespace _462

#endif
