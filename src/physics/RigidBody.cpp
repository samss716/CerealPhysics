
// integration of acceleration,velocity,position vectors
#include "physics/RigidBody.h"

#include <stdexcept>

// constructor
RigidBody::RigidBody(double mass, const Vector3 &initialPosition)
    : mass(mass),
      position(initialPosition),
      velocity(0.0),
      acceleration(0.0),
      accumForce(0.0),
      halfExtents(0.01, 0.01, 0.01),
      collisionRadius(0.01)
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
void RigidBody::integrate(double deltaTime) // calc new pos and vel based on accumulated forces; semi-implicit for numerical stability
{
    if (deltaTime <= 0.0)
    {
        throw std::invalid_argument("delta Time step must be positive.");
    }

    // Newton's Second Law: calculate acceleration from net accumulated forces
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

// generate bounding box from current pos
AABB RigidBody::getAABB() const
{
    return AABB(position, halfExtents);
}

// setting bounding box dimensions
void RigidBody::setHalfExtents(const Vector3 &halfExtents)
{
    if (halfExtents.x < 0.0 || halfExtents.y < 0.0 || halfExtents.z < 0.0)
    {
        throw std::invalid_argument("Half extents can not be negative.");
    }
    this->halfExtents = halfExtents;
}

// get collision sphere radius; lets PhysicsWorld retrieve radius.
double RigidBody::getCollisionRadius() const
{
    return collisionRadius;
}

// set collision sphere radius; adjust this for variously sized objs later
void RigidBody::setCollisionRadius(double radius) // sphere based collision
{
    if (radius < 0.0)
    {
        throw std::invalid_argument("Collision radius can not be negative.");
    }
    collisionRadius = radius;
}

void RigidBody::applyImpulse(const Vector3 &impulse)
{
    velocity += impulse / mass;
} // implementing impulse function to change velocity of rigid body based on impulse applied.

void RigidBody::translate(const Vector3 &displacement)
{
    position += displacement; // implementing translate function to allow PhysicsWorld to move body by specified displacement vector without exposing private variable. used to resolve collisions by moving bodies apart.
}