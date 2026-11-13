
#include <iostream>
#include <iomanip>

#include "physics/PhysicsWorld.h"

int main()
{

    // Simulation settings
    const double deltaTime = 1.0 / 120.0;
    const int totalSteps = 120;

    // Gravity acceleration (Y-up coordinate system)
    PhysicsWorld world(Vector3(0.0, -9.81, 0.0));

    // Create a cereal piece
    world.addBody(0.02, Vector3(0.0, 5.0, 0.0)); // mass = 0.02 kg , start pos : 5 meters above origin

    std::cout << "CerealPhysics initialized!\n";

    // Run the simulation
    for (int step = 0; step < totalSteps; step++)
    {
        world.step(deltaTime);
    }

    // retrieve sim cereal piece
    const RigidBody &cereal = world.getBody(0);

    // Display final results
    std::cout << std::fixed << std::setprecision(4);

    std::cout << "Simulation complete!\n\n";

    std::cout << "num of bodies: " << world.getBodyCount() << "\n";

    std::cout << "Final Position: "
              << cereal.getPosition().x << ", "
              << cereal.getPosition().y << ", "
              << cereal.getPosition().z << "\n";

    std::cout << "Final Velocity: "
              << cereal.getVelocity().x << ", "
              << cereal.getVelocity().y << ", "
              << cereal.getVelocity().z << "\n";

    return 0;
}