#pragma once


#include <glm/glm.hpp>
#include "mesh.h" 
#include "PhysicsObject.h"
#include <random>  // Ensure this is included for std::default_random_engine and std::uniform_real_distribution
#include <vector>
#include <unordered_map>

// Fwd declaration
class MeshDb;
class ShaderDb;
class Camera;

class PhysicsEngine
{
public:
	bool dragEnabled = true;  // Initially, drag force is enabled 
	unsigned int initialSeed; 

	struct GridCell {
		std::vector<Particle*> particles;
	};
	// Constructor
	PhysicsEngine()
		: gridOrigin(0.0f, 0.0f, 0.0f), // Initialize gridOrigin with proper coordinates 
		gridSizeX(gridSize), 
		gridSizeZ(gridSize),
		generator(static_cast<unsigned int>(std::time(nullptr))) // Initialize the generator
	{
		// Initialize the 3D grid
		grid.resize(gridSizeX, std::vector<std::vector<GridCell>>(1, std::vector<GridCell>(gridSizeZ)));
	}
	// Method to handle collisions within a grid cell
	void HandleCollisionsWithinCell(GridCell& cell, float deltaTime); // Add this line 
	void Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb);
	void Update(float deltaTime);
	void Display(const glm::mat4& viewMatrix, const glm::mat4& projMatrix);
	void HandleInputKey(int keyCode, bool pressed);

	void Task1Init();
	void Task1Update(float deltaTime, float totalTime);

	

	// Method to add a particle to the grid
	void AddParticleToGrid(Particle& particle) {
		int x = (int)((particle.Position().x - gridOrigin.x) / cellSize + 0.5f);
		int z = (int)((particle.Position().z - gridOrigin.z) / cellSize + 0.5f);
		x = std::max(0, std::min(x, gridSizeX - 1));
		z = std::max(0, std::min(z, gridSizeZ - 1));

		grid[x][0][z].particles.push_back(&particle);
	}
	// Method to clear the grid
	void ClearGrid() {
		for (auto& row : grid)
			for (auto& layer : row)
				for (auto& cell : layer)
					cell.particles.clear();
	}


private:
	const int gridSize = 10; // Number of cells along one dimension
	const float cellSize = 30.0f; // Each cell has dimensions 30x30 units
	const glm::vec3 gridOrigin;

	
	// 3D grid to store particles
	std::vector<std::vector<std::vector<GridCell>>> grid;
	int gridSizeX = gridSize, gridSizeZ = gridSize;
	
	
	std::default_random_engine generator;  // Random engine as a member
	
	bool useNewSeed = false;  // Determines if a new seed should be used on reset

	//std::vector<Particle> particles;
	PhysicsBody ground, sphere; 


	RigidBody rbody1;
};