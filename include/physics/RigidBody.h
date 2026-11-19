#pragma once

#include "math/Vector3.h"
#include "collision/AABB.h"
// utilizes encapsulation
class RigidBody
{
private:
    // physics properties
    double mass;
    double collisionRadius; // for collision detection

    // linear motion
    Vector3 position;
    Vector3 velocity;
    Vector3 acceleration;
    Vector3 halfExtents; // half extents of bounding box for collision detection

    // Forces acting on body
    Vector3 accumForce;

public:
    // Constructor
    RigidBody(double mass, const Vector3 &initialPosition);

    double getCollisionRadius() const;      // getter for collision radius
    void setCollisionRadius(double radius); // setter for collision radius

    // apply force to body
    void applyForce(const Vector3 &force);

    // update velocity and pos
    void integrate(double deltaTime);

    void applyImpulse(const Vector3 &impulse); // impulse function

    // accessor getter
    double getMass() const;
    const Vector3 &getPosition() const; // passing vec by reference w/out copying. prevents caller function from modifying rigid body
    const Vector3 &getVelocity() const;
    const Vector3 &getAcceleration() const;

    // getter for collision detection
    AABB getAABB() const;                            // returns bounding box of rigid body for collision detection
    void setHalfExtents(const Vector3 &halfExtents); // set half extents of bounding box for collision detection
};