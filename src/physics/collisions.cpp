#include "physics/collisions.hpp"

namespace _462
{
    bool collides(SphereBody &body1, SphereBody &body2, real_t collision_damping)
    {
        // TODO detect collision. If there is one, update velocity
        if (relative_velocity(body1, body2.velocity, body2.position) > 0)
        {
            real_t distance = length(body2.position - body1.position);
            if (distance >= body1.radius + body2.radius)
                return false;
            // 按照文档中公式计算速度
            real_t m1 = body1.mass, m2 = body2.mass;
            Vector3 d = (body2.position - body1.position) / distance;
            Vector3 v1 = body1.velocity, v2 = body2.velocity, v1_new, v2_new;
            v2_new = v2 + 2 * d * (m1 / (m1 + m2)) * dot(v1 - v2, d);
            v1_new = (m1 * v1 + m2 * v2 - m2 * v2_new) / m1;
            // 更新速度
            body1.velocity = damping(v1_new, collision_damping);
            body2.velocity = damping(v2_new, collision_damping);
            return true;
        }
        return false;
    }

    bool collides(SphereBody &body1, TriangleBody &body2, real_t collision_damping)
    {
        // TODO detect collision. If there is one, update velocity

        Vector3 va = body2.vertices[0], vb = body2.vertices[1], vc = body2.vertices[2];
        // calculate normal
        Vector3 n = normalize(cross(vb - va, vc - va));
        real_t d = dot(body1.position - body2.position, n);
        Vector3 projection_point = body1.position - d * n;

        if (relative_velocity(body1, body2.velocity, projection_point) >= 0)
        {
            if (abs(d) <= body1.radius && is_in_triangle(projection_point, body2.vertices))
            {
                // 按照和平面相撞的公式计算速度
                Vector3 v_new = body1.velocity - 2 * dot(body1.velocity, n) * n;
                // 更新速度
                body1.velocity = damping(v_new, collision_damping);
                return true;
            }
        }
        return false;
    }

    bool collides(SphereBody &body1, PlaneBody &body2, real_t collision_damping)
    {
        // TODO detect collision. If there is one, update velocity
        Vector3 v = body1.velocity, n = body2.normal;
        real_t d = dot(body1.position - body2.position, n);
        if (relative_velocity(body1, body2.velocity, body1.position - d * n) > 0)
        {
            if (abs(d) > body1.radius)
                return false;
            // 按照文档中公式计算速度
            Vector3 v_new = v - 2 * dot(v, n) * n;
            // 更新速度
            body1.velocity = damping(v_new, collision_damping);
            return true;
        }
        return false;
    }

    real_t relative_velocity(SphereBody &body1, Vector3 velocity, Vector3 position)
    {
        return dot(normalize(body1.velocity - velocity), normalize(position - body1.position));
    }

    // 碰撞后减速
    Vector3 damping(Vector3 velocity, real_t collision_damping)
    {
        // 过于小的速度直接认为是0
        if (length(velocity) < 1e-6)
        {
            std::cout << "Velocity too small";
            return Vector3::Zero;
        }

        return velocity - collision_damping * velocity;
    }

    bool is_in_triangle(const Vector3 point, const Vector3 vertices[3])
    {
        // compute vector
        Vector3 v0 = vertices[2] - vertices[0];
        Vector3 v1 = vertices[1] - vertices[0];
        Vector3 v2 = point - vertices[0];
        // compute dot products
        real_t dot00 = dot(v0, v0);
        real_t dot01 = dot(v0, v1);
        real_t dot02 = dot(v0, v2);
        real_t dot11 = dot(v1, v1);
        real_t dot12 = dot(v1, v2);
        // Compute barycentric coordinates
        real_t invDenom = real_t(1) / (dot00 * dot11 - dot01 * dot01);
        real_t beta = (dot00 * dot12 - dot01 * dot02) * invDenom;
        real_t gamma = (dot11 * dot02 - dot01 * dot12) * invDenom;
        // Check if point is in triangle
        return (beta >= 0) && (gamma >= 0) && (beta + gamma < 1);
    }

} // namespace _462
