
#include "physics/RigidBody.h"

#include <stdexcept>

// constructor
RigidBody::RigidBody(double mass, const Vector3 &initialPosition)
    : mass(mass),
      position(initialPosition),
      velocity(0.0),
      acceleration(0.0),
      accumForce(0.0)
{
    if (mass <= 0.0)
    {
        throw std::invalid_argument("RigidBody mass must be positive.");
    }
}

// apply force to body
void RigidBody::applyForce(const Vector3 &force)
{
    accumForce += force;
}

// update the body's motion
void RigidBody::integrate(double deltaTime)
{
    if (deltaTime <= 0.0)
    {
        throw std::invalid_argument("delta Time step must be positive.");
    }

    // Newton's Second Law: F = ma, rearranged to a = F / m; applying gravity as flexible force from main.cpp
    acceleration = accumForce / mass;

    // semi-implicit Euler integration
    velocity += acceleration * deltaTime;
    position += velocity * deltaTime;

    // reset forces for the next simulation step
    accumForce = Vector3(0.0);
}

// getters
double RigidBody::getMass() const
{
    return mass;
}

const Vector3 &RigidBody::getPosition() const
{
    return position;
}

const Vector3 &RigidBody::getVelocity() const
{
    return velocity;
}

const Vector3 &RigidBody::getAcceleration() const
{
    return acceleration;
}
