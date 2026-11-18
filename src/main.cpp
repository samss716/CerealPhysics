
#include <iostream>
#include <iomanip>
#include "collision/AABB.h"

#include "physics/PhysicsWorld.h"

int main()
{

    // Simulation settings
    const double fixedTimeStep = 1.0 / 120.0; // fixed physics timestep
    const double simulationDuration = 1.0;

    // create physics world
    PhysicsWorld world(Vector3(0.0, -9.81, 0.0));

    // Create a cereal piece
    world.addBody(0.02, Vector3(0.0, 5.0, 0.0)); // mass = 0.02 kg , start pos : 5 meters above origin

    std::cout << "CerealPhysics initialized!\n";

    double accumulator = 0.0;
    double elapsedTime = 0.0;

    // Run the simulation
    while (elapsedTime < simulationDuration)
    {
        const double frameTime = 1.0 / 60.0;

        accumulator += frameTime;
        elapsedTime += frameTime;

        while (accumulator >= fixedTimeStep)
        {
            world.step(fixedTimeStep);
            accumulator -= fixedTimeStep;
        }
    }

    // retrieve sim cereal piece
    const RigidBody &cereal = world.getBody(0);

    // Display final results
    std::cout << std::fixed << std::setprecision(4);

    std::cout << "Simulation complete!\n\n";

    std::cout << "Final Position: "
              << cereal.getPosition().x << ", "
              << cereal.getPosition().y << ", "
              << cereal.getPosition().z << "\n";

    std::cout << "Final Velocity: "
              << cereal.getVelocity().x << ", "
              << cereal.getVelocity().y << ", "
              << cereal.getVelocity().z << "\n";

    // collision detection test
    // create two bounding boxes
    AABB boxA(Vector3(0.0, 0.0, 0.0), Vector3(0.5, 0.5, 0.5)); // center at origin, half extents of 0.5 in each direction
    AABB boxB(Vector3(2.0, 0.0, 0.0), Vector3(0.5, 0.5, 0.5)); // center at origin, half extents of 0.5 in each direction, shifted along x-axis by 0.75

    // check if they overlap
    if (boxA.overlaps(boxB))
    {
        std::cout << "The two bounding boxes overlap!\n";
    }
    else
    {
        std::cout << "The two bounding boxes do not overlap.\n";
    }

    return 0;
}