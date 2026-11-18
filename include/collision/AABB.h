#pragma once

#include "math/Vector3.h"

class AABB
{
private: // min and max corners of bounding box
    Vector3 min;
    Vector3 max;

public:
    // construct a bounding box from its center and half-size
    AABB(const Vector3 &center, const Vector3 &halfExtents); // half extents defines distance from the center to each face of the box

    // checking whether two AABBs overlap
    bool overlaps(const AABB &other) const;

    // access bounding box boundaries
    const Vector3 &getMin() const;
    const Vector3 &getMax() const;
};
