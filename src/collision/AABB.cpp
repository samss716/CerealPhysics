#include "collision/AABB.h"

#include <stdexcept>

// constructor
AABB::AABB(const Vector3 &center, const Vector3 &halfExtents)
{
    if (halfExtents.x < 0.0 || halfExtents.y < 0.0 || halfExtents.z < 0.0)
    {
        throw std::invalid_argument("Half extents can not be negative.");
    }

    min = center - halfExtents;
    max = center + halfExtents;
}

// checking whether two AABBs overlap
bool AABB::overlaps(const AABB &other) const
{
    // Check for overlap along each axis
    bool overlapX = (min.x <= other.max.x && max.x >= other.min.x);
    bool overlapY = (min.y <= other.max.y && max.y >= other.min.y);
    bool overlapZ = (min.z <= other.max.z && max.z >= other.min.z);

    return overlapX && overlapY && overlapZ;
}

// get minimum corner
const Vector3 &AABB::getMin() const
{
    return min;
}

// get maximum corner
const Vector3 &AABB::getMax() const
{
    return max;
}