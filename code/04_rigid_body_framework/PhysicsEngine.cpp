#include "PhysicsEngine.h"
#include "Application.h"
#include "Camera.h"
#include "Force.h"
#include <cstdlib> // For rand() and srand()
#include <ctime>   // For time()
#include <string>
#include <iostream>

#include <glm/gtx/matrix_cross_product.hpp>
#include <glm/gtx/orthonormalize.hpp>
#include <glm/ext.hpp> 
#include <glm/gtx/string_cast.hpp>

using namespace glm;

const glm::vec3 GRAVITY = glm::vec3(0, -9.81, 0);

// PARTICLE_RADIUS 
const float PARTICLE_RADIUS = 1.0f; // Adjustable as needed

const int GRID_SIZE = 10;
// Adjusted for 3 grids of 10x10 particles each
const int TOTAL_PARTICLES = GRID_SIZE * GRID_SIZE * 3;
Particle particles[TOTAL_PARTICLES];




void ExplicitEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	// Apply the impulse to velocity directly. Impulse is force applied over a short time.
	// Since impulse = force * deltaTime and force = mass * acceleration,
	// the change in velocity (deltaV) due to impulse can be calculated as impulse / mass.
	vec3 deltaV = impulse / mass;

	// Update velocity with the effect of the impulse and acceleration
	vel += deltaV + (accel * dt);

	// Update position based on the new velocity
	pos += vel * dt;
}

void SymplecticEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{

	// Using the integrator, compute the new velocity (vel+1)
	vel = vel + dt * accel;

	// Using the integrator, compute the new position (pos+1)
	pos = pos + dt * vel;
}



vec3 GroundCollisionResponse(Particle& particle, float groundY, float coefficientOfRestitution) {
	glm::vec3 particlePos = particle.Position();
	glm::vec3 velocity = particle.Velocity();
	glm::vec3 impulse = glm::vec3(0.0f);

	// Check if the particle is below the ground level
	if (particlePos.y < groundY + PARTICLE_RADIUS) {
		// Reflect the y component of the velocity
		velocity.y = -velocity.y * coefficientOfRestitution;

		// Adjust the particle's position to be on the surface
		particlePos.y = groundY + PARTICLE_RADIUS;

		// Calculate impulse based on the change in velocity
		impulse.y = particle.Mass() * (velocity.y - particle.Velocity().y);
	}

	// Update particle's velocity and position
	particle.SetVelocity(velocity);
	particle.SetPosition(particlePos);

	return impulse;
}



void UpdateAngularIntegration(RigidBody& rb, float deltaTime) {
	auto newAngVel = rb.AngularVelocity() + deltaTime * rb.AngularAcceleration();
	rb.SetAngularVelocity(newAngVel);

	mat3 angVelSkew = matrixCross3(newAngVel);
	mat3 R = mat3(rb.Orientation());
	R += deltaTime * angVelSkew * R;
	R = glm::orthonormalize(R);

	rb.SetOrientation(glm::mat4(R));
}



// This is called once
void PhysicsEngine::Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	// Get a few meshes/shaders from the databases
	auto defaultShader = shaderDb.Get("default");
	auto groundMesh = meshDb.Get("plane");
	auto sphereMesh = meshDb.Get("sphere");


	meshDb.Add("cube", Mesh(MeshDataFromWavefrontObj("resources/models/cube.obj")));
	meshDb.Add("sphere", Mesh(MeshDataFromWavefrontObj("resources/models/sphere.obj")));
	meshDb.Add("cone", Mesh(MeshDataFromWavefrontObj("resources/models/cone.obj")));
	auto mesh = meshDb.Get("sphere");

	//distanc ebetween particles
	//float separation = 0.5f;

	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(30.0f));
	ground.SetPosition(vec3(0.0f, 0.0f, 0.0f));

	// Initialise cube
	sphere.SetMesh(sphereMesh);
	sphere.SetShader(defaultShader);
	sphere.SetScale(vec3(1.0f));  // Adjust the cube to match the ground's scale
	sphere.SetPosition(vec3(0.0f, 0.0f, 0.0f));  // Center the cube at the same position as the ground

	srand(static_cast<unsigned int>(time(0)));

	// Sphere type definitions
	vec4 colors[] = { vec4(1, 0, 0, 1), vec4(0, 1, 0, 1), vec4(0, 0, 1, 1) };
	float masses[] = { 1.0f, 2.0f, 3.0f };
	const float PARTICLE_RADIUS = 1.0f; // Sphere radius

	// Plane boundaries for random spawning
	float minX = -15.0f, maxX = 15.0f;
	float minZ = -15.0f, maxZ = 15.0f;
	float initialY = 0.1f; // Slightly above the plane to ensure they are visible

	for (int i = 0; i < TOTAL_PARTICLES; ++i) {
		// Randomly pick a type
		int type = rand() % 3;

		// Generate random positions within the plane boundaries
		float posX = minX + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (maxX - minX)));
		float posZ = minZ + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (maxZ - minZ)));

		particles[i].SetMesh(meshDb.Get("sphere"));
		particles[i].SetShader(shaderDb.Get("default"));
		particles[i].SetColor(colors[type]);
		particles[i].SetScale(vec3(PARTICLE_RADIUS));
		particles[i].SetPosition(vec3(posX, initialY, posZ));
		particles[i].SetVelocity(vec3(0.0f));
		particles[i].SetMass(masses[type]);
	}



	camera = Camera(vec3(5, 10, 20));
}




// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime) {
	static double accumulator = 0.0;
	const double fixedDeltaTime = 0.016; // 16ms for a 60Hz update rate 


	float kd = 0.1f; // Damping constant 


	accumulator += deltaTime;

	while (accumulator >= fixedDeltaTime) {
		// Clear forces and impulses for all particles
		for (int i = 0; i < GRID_SIZE * GRID_SIZE; ++i) {
			Force::Gravity(particles[i]);
		}



		// Symplectic Euler integration using SetPosition and SetVelocity
		for (int i = 0; i < GRID_SIZE * GRID_SIZE; ++i) {
			// Get the current state of the particle
			vec3 p = particles[i].Position();
			vec3 v = particles[i].Velocity();
			vec3 acceleration = particles[i].AccumulatedForce() / particles[i].Mass();

			// Use the Symplectic Euler to update velocity and position
			SymplecticEuler(p, v, particles[i].Mass(), acceleration, vec3(0.0f), fixedDeltaTime);

			// Apply the updated state back to the particle
			particles[i].SetPosition(p);
			particles[i].SetVelocity(v);

			// Handle ground collision response or other interactions as necessary
			GroundCollisionResponse(particles[i], 0.0f, 0.9f);
		}



		accumulator -= fixedDeltaTime;
	}
}





// This is called every frame, after Update
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{

	for (int i = 0; i < 100; i++) {
		particles[i].Draw(viewMatrix, projMatrix);
	}

	ground.Draw(viewMatrix, projMatrix);
}

void PhysicsEngine::HandleInputKey(int keyCode, bool pressed)
{
	switch (keyCode)
	{
	default:
		break;
	}
}