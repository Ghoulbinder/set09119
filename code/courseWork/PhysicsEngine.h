#pragma once


#include <glm/glm.hpp>
#include "mesh.h" 
#include "PhysicsObject.h"
#include <random>  // Ensure this is included for std::default_random_engine and std::uniform_real_distribution


// Fwd declaration
class MeshDb;
class ShaderDb;
class Camera;

class PhysicsEngine
{
public:

	void Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb);
	void Update(float deltaTime);
	void Display(const glm::mat4& viewMatrix, const glm::mat4& projMatrix);
	void HandleInputKey(int keyCode, bool pressed);

	void Task1Init(); 
	void Task1Update(float deltaTime, float totalTime); 
	
	bool determineIfNewSeedIsRequired(); 
	// ... rest of the tasks here

	


private:
	float elapsedTime; // This could be a timer or other condition variable 
	
	std::default_random_engine generator;  // Random engine as a member
	bool needReset = false;  // Tracks whether a reset is needed
	bool useNewSeed = false;  // Determines if a new seed should be used on reset

	//std::vector<Particle> particles;
	PhysicsBody ground, sphere; 


	RigidBody rbody1;
};