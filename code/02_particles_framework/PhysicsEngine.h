#pragma once


#include <glm/glm.hpp>

#include "PhysicsObject.h"

// Fwd declaration
class MeshDb;
class ShaderDb;
class Camera;

class PhysicsEngine
{
public:

	const double dt = 1.0 / 60.0; // Fixed time step
	void Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb);
	void Update(float deltaTime, float totalTime);
	void Display(const glm::mat4& viewMatrix, const glm::mat4& projMatrix);
	void HandleInputKey(int keyCode, bool pressed);

private:

	PhysicsBody ground;
	PhysicsBody cube;
	Particle particle, particle2, particle3, particle4;

	//const double dt = 1.0 / 60.0; // Fixed time step
	double currentTime;
	double accumulator = 0.0;

};