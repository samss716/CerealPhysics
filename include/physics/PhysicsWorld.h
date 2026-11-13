// Manages gravity, rigid bodies, and simulation updates
// 1. Store gravity as a vector.
// 2. Keep track of all rigid bodies in the simulation.
// 3. Apply gravity to each body.
// 4. Advance every body by one timestep.

#pragma once

#include "physics/RigidBody.h"
#include <vector>
#include <cstddef>

class PhysicsWorld
{
private:
    // Global gravity applied ONCE to all bodies rather than individual
    Vector3 gravity;

    // All rigid bodies in the simulation; allows for dynamic growth
    std::vector<RigidBody> bodies;

public:
    // Constructor
    PhysicsWorld(const Vector3 &gravity);

    // adding new rigid body in world
    void addBody(double mass, const Vector3 &initialPosition);

    // Advance sim by one timestep
    void step(double deltaTime);

    // access body by its index
    RigidBody &getBody(std::size_t index);
    const RigidBody &getBody(std::size_t index) const;

    // getter for number of bodies in world
    std::size_t getBodyCount() const;
};