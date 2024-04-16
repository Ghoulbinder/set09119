#include "SpatialGrid.h"

void SpatialGrid::ClearGrid() {
    grid_.clear();
}

GridCellKey SpatialGrid::CalculateCellKey(const glm::vec3& position) const {
    return { static_cast<int>(floor(position.x / cellSize_)),
             static_cast<int>(floor(position.y / cellSize_)),
             static_cast<int>(floor(position.z / cellSize_)) };
}

void SpatialGrid::UpdateGrid(const std::vector<PhysicsBody*>& bodies) {
    ClearGrid();
    for (PhysicsBody* body : bodies) {
        GridCellKey key = CalculateCellKey(body->Position());
        grid_[key].push_back(body);
    }
}

std::vector<std::pair<PhysicsBody*, PhysicsBody*>> SpatialGrid::GetPossibleCollisions() const {
    std::vector<std::pair<PhysicsBody*, PhysicsBody*>> collisions;
    for (auto& cell : grid_) {
        const auto& bodies = cell.second;
        for (size_t i = 0; i < bodies.size(); i++) {
            for (size_t j = i + 1; j < bodies.size(); j++) {
                collisions.emplace_back(bodies[i], bodies[j]);
            }
        }
    }
    return collisions;
}
