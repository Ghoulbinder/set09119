#pragma once
#ifndef OBB_H
#define OBB_H

#include <glm/glm.hpp>

// Forward declaration if needed
class PhysicsObjectWithOBB;

// Define the structure for Oriented Bounding Box
struct OBB {
    glm::vec3 position;
    glm::mat3 orientation;
    glm::vec3 halfExtents;

    // Method for collision detection with another OBB
    bool Intersects(const OBB& other);
};

#endif // OBB_H
