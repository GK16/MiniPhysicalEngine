#include "physics/physics.hpp"

namespace _462
{

    Physics::Physics()
    {
        reset();
    }

    Physics::~Physics()
    {
        reset();
    }

    void Physics::step(real_t dt)
    {
        // TODO step the world forward by dt. Need to detect collisions, apply
        // forces, and integrate positions and orientations.
        //
        // Note: put RK4 here, not in any of the physics bodies
        //
        // Must use the functions that you implemented
        //
        // Note, when you change the position/orientation of a physics object,
        // change the position/orientation of the graphical object that represents
        // it
        detect_collisions();
        for (size_t i = 0; i < num_spheres(); ++i)
        {
            SphereBody *body = spheres[i];
            body->I = 0.4f * body->mass * body->radius * body->radius;

            State state;
            state.position = body->position;
            state.velocity = body->velocity;
            state.orientation = body->orientation;
            state.angular_velocity = body->angular_velocity;
            apply_forces(body, state);

            State k1;
            k1.position = state.velocity * dt;
            k1.velocity = body->force / body->mass * dt;
            k1.angular_velocity = body->torque / body->I * dt;
            State k2 = estimate(body, state, k1, dt, 0.5);
            State k3 = estimate(body, state, k2, dt, 0.5);
            State k4 = estimate(body, state, k3, dt, 1);

            body->position += k1.position / 6.0 + k2.position / 3.0 + k3.position / 3.0 + k4.position / 6.0;
            body->velocity += k1.velocity / 6.0 + k2.velocity / 3.0 + k3.velocity / 3.0 + k4.velocity / 6.0;
            body->angular_velocity += k1.angular_velocity / 6.0 + k2.angular_velocity / 3.0 + k3.angular_velocity / 3.0 + k4.angular_velocity / 6.0;
            body->orientation = get_new_orientation(body->orientation, body->angular_velocity, dt);

            body->sphere->position = body->position;
            body->sphere->orientation = body->orientation;
        }

    }

    // Runge-Kutta
    State Physics::estimate(SphereBody *body, const State &old_state, const State &old_delta, real_t dt, real_t step)
    {
        State new_state;
        new_state.position = old_state.position + old_delta.position * step;
        new_state.velocity = old_state.velocity + old_delta.velocity * step;
        new_state.orientation = get_new_orientation(old_state.orientation, old_state.angular_velocity, dt * step);
        new_state.angular_velocity = old_state.angular_velocity + old_delta.angular_velocity * step;
        apply_forces(body, new_state);

        State new_delta;
        new_delta.position = new_state.velocity * dt;
        new_delta.velocity = body->force / body->mass * dt;
        new_delta.angular_velocity = body->torque / body->I * dt;
        return new_delta;
    }

    Quaternion Physics::get_new_orientation(Quaternion orientation, Vector3 angular_velocity, real_t dt)
    {
        if (length(angular_velocity * dt) > 0)
            return orientation * Quaternion(normalize(angular_velocity), length(angular_velocity * dt));
        else
            return orientation;
    }

    void Physics::detect_collisions()
    {
        for (size_t i = 0; i < num_spheres(); ++i)
        {
            // 检查球之间是否有碰撞
            for (size_t j = i + 1; j < num_spheres(); ++j)
            {
                collides(*spheres[i], *spheres[j], collision_damping);
            }
            // 检查球和三角之间是否有碰撞
            for (size_t j = 0; j < num_triangles(); ++j)
            {
                collides(*spheres[i], *triangles[j], collision_damping);
            }
            // 检查球和平面之间是否有碰撞
            for (size_t j = 0; j < num_planes(); ++j)
            {
                collides(*spheres[i], *planes[j], collision_damping);
            }
        }
    }

    void Physics::apply_forces(SphereBody *body, const State &state)
    {
        // 清除之前的力
        body->clear_force();
        // 加重力
        body->apply_force(gravity, Vector3::Zero);
        // 加弹簧
        for (size_t i = 0; i < num_springs(); ++i)
        {
            if (springs[i]->body1->id == body->id)
            {
                springs[i]->step(state);
            }
        }
    }

    void Physics::add_sphere(SphereBody *b)
    {
        spheres.push_back(b);
    }

    size_t Physics::num_spheres() const
    {
        return spheres.size();
    }

    void Physics::add_plane(PlaneBody *p)
    {
        planes.push_back(p);
    }

    size_t Physics::num_planes() const
    {
        return planes.size();
    }

    void Physics::add_triangle(TriangleBody *t)
    {
        triangles.push_back(t);
    }

    size_t Physics::num_triangles() const
    {
        return triangles.size();
    }

    void Physics::add_spring(Spring *s)
    {
        springs.push_back(s);
    }

    size_t Physics::num_springs() const
    {
        return springs.size();
    }

    void Physics::reset()
    {
        for (SphereList::iterator i = spheres.begin(); i != spheres.end(); i++)
        {
            delete *i;
        }
        for (PlaneList::iterator i = planes.begin(); i != planes.end(); i++)
        {
            delete *i;
        }
        for (TriangleList::iterator i = triangles.begin(); i != triangles.end(); i++)
        {
            delete *i;
        }
        for (SpringList::iterator i = springs.begin(); i != springs.end(); i++)
        {
            delete *i;
        }

        spheres.clear();
        planes.clear();
        triangles.clear();
        springs.clear();

        gravity = Vector3::Zero;
        collision_damping = 0.0;
    }

} // namespace _462
