#pragma once

#include "math/Vector3.h"
// utilizes encapsulation
class RigidBody
{
private:
    // physics properties
    double mass;

    // linear motion
    Vector3 position;
    Vector3 velocity;
    Vector3 acceleration;

    // Forces acting on body
    Vector3 accumForce;

public:
    // Constructor
    RigidBody(double mass, const Vector3 &initialPosition);

    // apply force to body
    void applyForce(const Vector3 &force);

    // update velocity and pos
    void integrate(double deltaTime);

    // accessor getter
    double getMass() const;
    const Vector3 &getPosition() const; // passing vec by reference w/out copying. prevents caller function from modifying rigid body
    const Vector3 &getVelocity() const;
    const Vector3 &getAcceleration() const;
};