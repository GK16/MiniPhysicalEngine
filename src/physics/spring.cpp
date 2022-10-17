#include "math/math.hpp"
#include "math/vector.hpp"
#include "math/quaternion.hpp"
#include "math/matrix.hpp"
#include "physics/spring.hpp"
#include "physics/body.hpp"
#include "physics/spherebody.hpp"
#include <iostream>

namespace _462
{

    Spring::Spring()
    {
        body1_offset = Vector3::Zero;
        body2_offset = Vector3::Zero;
        damping = 0.0;
    }

    void Spring::step(const State &state)
    {
        Vector3 offset1 = state.orientation * body1_offset;
        Vector3 offset2 = body2->orientation * body2_offset;
        // compute the attachment points
        Vector3 point1 = state.position + offset1;
        Vector3 point2 = body2->position + offset2;
        // compute directions for both bodies
        Vector3 dir1 = point1 - point2;
        Vector3 dir2 = -dir1;
        // compute the distance between two bodies
        real_t dist = length(point1 - point2);
        // compute relative velocity of both bodies
        real_t vel1 = dot(state.velocity, normalize(dir1));
        real_t vel2 = dot(body2->velocity, normalize(dir2));
        // compute force for both bodies
        Vector3 force1 = (-constant * (length(dir1) - equilibrium) - damping * vel1) * normalize(dir1);
        Vector3 force2 = (-constant * (length(dir2) - equilibrium) - damping * vel2) * normalize(dir2);
        // apply force
        body1->apply_force(force1, offset1);
        body2->apply_force(force2, offset2);
        // TODO apply forces to attached bodies
    }

} // namespace _462
