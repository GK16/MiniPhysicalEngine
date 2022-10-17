# MiniPhysicalEngine
by Guikai Huang

## Project Overview

- According to the formula given by the material, the simulation of the motion of the object and the applied force and torque can be realized
- Use quaternions to describe the problem of sphere rotation
- Use the integrator to implement the Runge-Kutta 4th-order equation according to the formula given by the material to integrate the position and direction of the sphere and solve the initial value problem
- Sphere and sphere collision detection: the distance between the center points of the two spheres is less than the sum of the radii, there is a collision, and the speed is recalculated according to the formula
- Sphere and plane collision detection: Project the center of the sphere onto the plane, and use a point on the plane and the normal of the plane to detect the collision between the sphere and the plane
- Sphere and triangle collision detection: first check whether the projection of the center of the sphere is inside the triangle, if so, use the sphere and plane collision
- Spring: applies a force (based on Hooke's law) to two objects attached to a spring
- Damping: Determine whether to reduce the fixed speed component per unit time

## Code Structure

-physics/spherebody.cpp
  - step_position(): returns the position change
  - step_orientation(): returns the angle change
  - apply_force(): Save the force and moment of the object at a certain time, which is convenient for subsequent calculation calls
- physics/collisions.cpp
  - Three versions of the collision function collides(), all calculated according to the formulas given in the material: sphere-to-sphere collision, sphere-to-triangle collision, sphere-to-plane collision
  - Auxiliary functions: is_in_triangle(), relative_velocity()
  - Slow down after collision: damping()
-physics/spring.cpp
  - step function step() , which applies a force (based on Hooke's law) to two bodies attached to a spring, along with damping
- physics/physics.cpp global step function step(), including previously implemented
  - detect_collisions() detects if collisions have occurred
  - apply_forces() assigns forces and moments to each object
  - estimate() Runge-Kutta-4th order equation
  - get_new_orientation() calculates the orientation quaternion

## Output

- In the collision and collision_stress scenes, the balls and the plane can collide normally (under MacOS, in the collision_stress scene, the small ball will flicker frequently, please ask a senior to run it under his Linux, and it will display normally)
- In the damping_test scenario, gravity and damping can be implemented
- In the newtons_cradle scene, Newton's pendulum moves under the action of gravity and springs
- In the rotation_test scenario, the ball rotates normally
- spring_rotation, the ball moves under the action of gravity and spring

## Lab Environment

- Operating system: MacOS 10.14, ubuntu16.04 (MacOS 10.15 has many problems)
- Development environment: Vscode editing, Cmake compilation