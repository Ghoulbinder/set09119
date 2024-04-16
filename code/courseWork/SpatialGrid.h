#pragma once

#include <unordered_map>
#include <vector>
#include <glm/glm.hpp>
#include "Physicsengine.h"  // This include is necessary because we are using PhysicsBody

struct GridCellKey {
    int x, y, z;
    bool operator==(const GridCellKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

namespace std {
    template<> struct hash<GridCellKey> {
        size_t operator()(const GridCellKey& k) const {
            return ((hash<int>()(k.x) ^ (hash<int>()(k.y) << 1)) >> 1) ^ (hash<int>()(k.z) << 1);
        }
    };
};

class SpatialGrid {
public:
    SpatialGrid(float cellSize) : cellSize_(cellSize) {}
    void UpdateGrid(const std::vector<PhysicsBody*>& bodies);
    std::vector<std::pair<PhysicsBody*, PhysicsBody*>> GetPossibleCollisions() const;

private:
    float cellSize_;
    std::unordered_map<GridCellKey, std::vector<PhysicsBody*>> grid_;
    void ClearGrid();
    GridCellKey CalculateCellKey(const glm::vec3& position) const;
};
