#pragma once


#include <glm/glm.hpp>

#include "PhysicsObject.h"

// Fwd declaration
class MeshDb;
class ShaderDb;
class Camera;

using namespace glm;

class PhysicsEngine
{
public:
	 
	//PhysicsEngine(); // Constructor declaration         
	static const int gridSize = 10; // Make gridSize a class constant or member 

	

	void Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb);
	void Update(float deltaTime, float totalTime);
	void Display(const glm::mat4& viewMatrix, const glm::mat4& projMatrix);
	void HandleInputKey(int keyCode, bool pressed);
	//void ApplyStructuralConstraint(Particle& p1, Particle& p2, float restLength); // Function declaration 

	void Task1Init();
	void Task1Update(float deltaTime, float totalTime); // 5-particle chain
	

private:


	double currentTime;
	double  accumulator = 0.0;
	const double fixedDeltaTime = 0.016; // 16ms for a 60Hz update rate// Fixed time step 
	PhysicsBody ground;
	PhysicsBody cube;
	//Particle particle[10];
	//const float PARTICLE_RADIUS = 0.1f; 
	//const int gridSize = 10; 
	//std::vector<Particle> particles; // Use this for all particles 
};
