#ifndef _462_PHYSICS_PHYSICS_HPP_
#define _462_PHYSICS_PHYSICS_HPP_

#include "math/math.hpp"
#include "math/quaternion.hpp"
#include "math/vector.hpp"
#include "physics/body.hpp"
#include "physics/spherebody.hpp"
#include "physics/trianglebody.hpp"
#include "physics/planebody.hpp"
#include "physics/spring.hpp"
#include "physics/collisions.hpp"

#include <vector>

namespace _462
{

    class Physics
    {
    public:
        Vector3 gravity;
        real_t collision_damping;

        Physics();
        ~Physics();

        void step(real_t dt);
        void add_sphere(SphereBody *s);
        size_t num_spheres() const;
        void add_plane(PlaneBody *p);
        size_t num_planes() const;
        void add_triangle(TriangleBody *t);
        size_t num_triangles() const;
        void add_spring(Spring *s);
        size_t num_springs() const;

        void reset();

        // 自己定义的函数
        // 检测碰撞是否发生
        void detect_collisions();
        // 为每个物体分配力和力矩
        void apply_forces(SphereBody *body, const State &state);
        // Runge-Kutta-4
        State estimate(SphereBody *body, const State &old_state, const State &old_delta, real_t dt, real_t step);
        // 计算朝向四元数
        Quaternion get_new_orientation(Quaternion orientation, Vector3 angular_velocity, real_t dt);

    private:
        typedef std::vector<Spring *> SpringList;
        typedef std::vector<SphereBody *> SphereList;
        typedef std::vector<PlaneBody *> PlaneList;
        typedef std::vector<TriangleBody *> TriangleList;

        SpringList springs;
        SphereList spheres;
        PlaneList planes;
        TriangleList triangles;
    };

} // namespace _462

#endif
