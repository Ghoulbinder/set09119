#pragma once


#include <glm/glm.hpp>
#include "mesh.h" 
#include "PhysicsObject.h"

// Fwd declaration
class MeshDb;
class ShaderDb;
class Camera;

class PhysicsEngine
{
public:
	void Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb);
	void Update(float deltaTime, float totalTime);
	void Display(const glm::mat4& viewMatrix, const glm::mat4& projMatrix);
	void HandleInputKey(int keyCode, bool pressed);

	void Task1Init();
	void Task1Update(float deltaTime, float totalTime);

	




private:

		bool paused = false; // Flag to indicate if the simulation is paused 
	int scenario = 0; // Variable to store the current test scenario 
	float scenarioActivationTime = 0.0f; // Tracks when the scenario was activated
	bool isScenarioPending = false; // Indicates if we're waiting to execute a scenario action
	float totalTime = 0.0f; // You might already have something similar to track total elapsed time

	PhysicsBody ground, sphere;


	RigidBody rbody1;
};