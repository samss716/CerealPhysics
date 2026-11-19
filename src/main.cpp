
#include <iostream>
#include <iomanip>
#include "collision/AABB.h"

#include "physics/PhysicsWorld.h"

int main()
{

    // Simulation settings
    const double fixedTimeStep = 1.0 / 120.0; // fixed physics timestep
                                              // const double simulationDuration = 1.0;

    // create physics world
    PhysicsWorld world(Vector3(0.0, -9.81, 0.0));

    // Create a cereal piece
    world.addBody(0.02, Vector3(0.0, 5.0, 0.0)); // mass = 0.02 kg , start pos : 5 meters above origin
    world.addBody(0.02, Vector3(0.015, 5.0, 0.0));
    world.addBody(0.02, Vector3(0.10, 5.0, 0.0));

    std::cout << "CerealPhysics initialized!\n";

    // simulate one physics step
    world.step(fixedTimeStep);

    // detect overlapping bounding boxes broad phase
    auto potentialCollisions = world.detectCollisions();

    // narrow phase
    auto confirmedCollisions = world.detectNarrowPhaseCollisions();

    std::cout << "Number of bodies: " << world.getBodyCount() << "\n";
    std::cout << "Number of potential collisions: " << potentialCollisions.size() << "\n";
    std::cout << "Number of confirmed collisions: " << confirmedCollisions.size() << "\n";

    for (const auto &pair : confirmedCollisions)
    {
        std::cout << "Collision detected between body " << pair.first << " and body " << pair.second << "\n";
    }

    return 0;
}