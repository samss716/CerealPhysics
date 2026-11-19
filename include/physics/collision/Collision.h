#pragma once

#include "math/Vector3.h"
// gets collision info for impulse solver
struct CollisionInfo
{

    std::size_t bodyAIndex;
    std::size_t bodyBIndex;

    Vector3 normal;
    double penetration;
};