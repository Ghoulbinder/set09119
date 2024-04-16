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
const int TOTAL_PARTICLES = 1000;
Particle particles[TOTAL_PARTICLES];

const float dampingFactor = 0.95f; // Adjust as needed 
//const float coefficientOfRestitution = 0.7f; //  adjust as needed 
bool shouldPrint = false; // Set to false to disable printing

float coefficientOfRestitution = 0.9f; // Bounciness of the cube's walls 

void SymplecticEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{

	// Using the integrator, compute the new velocity (vel+1)
	vel = vel + dt * accel;

	// Using the integrator, compute the new position (pos+1)
	pos = pos + dt * vel;
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

void PhysicsEngine::HandleCollisionsWithinCell(GridCell& cell, float deltaTime) {
    // Check collisions within the grid cell
    for (size_t i = 0; i < cell.particles.size(); ++i) {
        Particle& p = *cell.particles[i];

        for (size_t j = i + 1; j < cell.particles.size(); ++j) {
            Particle& q = *cell.particles[j];
            vec3 displacement = q.Position() - p.Position();
            float distance = glm::length(displacement);
            float combinedRadius = PARTICLE_RADIUS * 2;

            if (distance < combinedRadius) { // Collision detected
                vec3 normal = glm::normalize(displacement);
                float penetration = combinedRadius - distance;

                // Calculate new velocities using the coefficient of restitution
                vec3 relativeVelocity = q.Velocity() - p.Velocity();
                float velocityAlongNormal = glm::dot(relativeVelocity, normal);
                if (velocityAlongNormal > 0) continue;

                float e = coefficientOfRestitution;
                float impulseMagnitude = -(1 + e) * velocityAlongNormal;
                impulseMagnitude /= (1 / p.Mass() + 1 / q.Mass());

                vec3 impulse = impulseMagnitude * normal;

                p.SetVelocity(p.Velocity() - impulse / p.Mass());
                q.SetVelocity(q.Velocity() + impulse / q.Mass());

                // Position correction to prevent sinking due to floating point precision issues
                vec3 correction = (penetration / (1 / p.Mass() + 1 / q.Mass())) * normal;
                p.SetPosition(p.Position() + correction / (2 * p.Mass()));
                q.SetPosition(q.Position() - correction / (2 * q.Mass()));
            }
        }

    }
}


// This is called once
void PhysicsEngine::Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{



	meshDb.Add("cube", Mesh(MeshDataFromWavefrontObj("resources/models/cube.obj")));
	meshDb.Add("sphere", Mesh(MeshDataFromWavefrontObj("resources/models/sphere.obj")));
	meshDb.Add("cone", Mesh(MeshDataFromWavefrontObj("resources/models/cone.obj")));

	// Get a few meshes/shaders from the databases 
	auto defaultShader = shaderDb.Get("default");
	auto groundMesh = meshDb.Get("cube");
	auto sphereMesh = meshDb.Get("sphere");
	auto mesh = meshDb.Get("sphere");

	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(300.0f, 1.0f, 300.0f));
	ground.SetPosition(vec3(0.0f, -1.0f, 0.0f));//changed the y to -1 so to account for the scale of 1 so the spheres remain on 0

	// Initialise cube
	sphere.SetMesh(sphereMesh);
	sphere.SetShader(defaultShader);

	//sphere.SetPosition(vec3(0.0f, 0.0f, 0.0f));  // Center the cube at the same position as the ground

	srand(static_cast<unsigned int>(time(0)));

	// Sphere type definitions
	vec4 colors[] = { vec4(1, 0, 0, 1), vec4(0, 1, 0, 1), vec4(0, 0, 1, 1) };
	float masses[] = { 1.0f, 2.0f, 3.0f };
	const float PARTICLE_RADIUS = 1.0f; // Sphere radius

	int numParticlesPerGrid = 1; // Adjust as needed
	int totalGrids = GRID_SIZE * GRID_SIZE; // Total number of grids

	//float gridSpacing = 30.0f; // Adjust as needed
	float initialY = PARTICLE_RADIUS;

	for (int i = 0; i < TOTAL_PARTICLES; ++i) {
		int type = rand() % 3; // Keep the random type if it applies to your simulation
		// Randomize positions within bounds 
		float posX = static_cast<float>(rand()) / RAND_MAX * 300.0f - 150.0f; // Random X between -15 and 15
		float posY = PARTICLE_RADIUS + 1.0f;
		float posZ = static_cast<float>(rand()) / RAND_MAX * 300.0f - 150.0f; // Random Z between -15 and 15


		// Randomize velocities between -20 and 20 m/sec for the x and z axes
		float velX = static_cast<float>(rand()) / RAND_MAX * 40.0f - 20.0f; // Random velocity X between -20 and 20
		// Assuming no specific requirement for Y velocity, set to a default or randomize as needed
		float velY = 0.0f; // Default Y velocity
		float velZ = static_cast<float>(rand()) / RAND_MAX * 40.0f - 20.0f; // Random velocity Z between -20 and 20
		// Randomize velocities to be between 1 and 3 for each axis
		//float velX = static_cast<float>(rand()) / RAND_MAX * 2.0f + 1.0f; // Random velocity X between 1 and 3
		//float velY = static_cast<float>(rand()) / RAND_MAX * 2.0f + 1.0f; 
		//float velZ = static_cast<float>(rand()) / RAND_MAX * 2.0f + 1.0f; // Random velocity Z between 1 and 3

		// Setup particle with the random position and velocity
		Particle& particle = particles[i];
		particle.SetPosition(vec3(posX, posY, posZ));
		particle.SetVelocity(vec3(velX, velY, velZ));

		// Spawn particle
		
		particle.SetMesh(meshDb.Get("sphere"));
		particle.SetShader(shaderDb.Get("default"));
		particle.SetColor(colors[type]);
		particle.SetScale(vec3(1.0f));
		particle.SetMass(masses[type]);

		// Add particle to grid
		AddParticleToGrid(particle); 
	}


	camera = Camera(vec3(10, 100, 100));
}


void PhysicsEngine::Task1Init() {
	if (useNewSeed) {
		// Reinitialize the random number generator with a new random seed
		std::random_device rd;
		generator.seed(rd());
	}
	else {
		// Reinitialize the random number generator with the current time as the seed
		generator.seed(static_cast<unsigned int>(std::time(nullptr)));
	}

	// Reinitialize particles using the selected seed
	std::uniform_real_distribution<float> positionDist(-15.0f, 15.0f);
	std::uniform_real_distribution<float> velocityDist(-20.0f, 20.0f);

	for (Particle& particle : particles) {
		glm::vec3 pos(positionDist(generator), positionDist(generator), positionDist(generator));
		glm::vec3 vel(velocityDist(generator), velocityDist(generator), velocityDist(generator));
		particle.SetPosition(pos);
		particle.SetVelocity(vel);

		AddParticleToGrid(particle);
	}
}


void PhysicsEngine::Task1Update(float deltaTime, float totalTime){
	// Handle particle updates, collisions, etc.
	// Optionally, check for a condition to reset with the same or a new seed
	if (needReset) {
		bool useNewSeed = determineIfNewSeedIsRequired();
		if (useNewSeed) {
			// Re-seed the generator
			static_cast<std::default_random_engine*>(nullptr)->seed(static_cast<unsigned int>(time(nullptr)));
		}
		Task1Init(); // Reinitialize particles with the new or same seed
	}



}
bool PhysicsEngine::determineIfNewSeedIsRequired() {
	// Reset the seed if more than 100 seconds have elapsed or 
	// if the user has requested a seed reset.
	return elapsedTime > 100.0f || needReset;
}


// This is called every frame
// This should be called every fixed time step, e.g., 0.016 seconds for 60Hz
///CHECK WITH BABIS IF I SHOULD USE DELTATIME INSTEAD OF FIXEDdELTATIME IN UPDATE LOOP
void PhysicsEngine::Update(const float deltaTime) {
	ClearGrid(); // Clear the grid for new assignments 
	elapsedTime += deltaTime;  // Keep track of the elapsed time 
	// Define properties of the environment
	const glm::vec3 cubeCentre = glm::vec3(0.0f, 0.0f, 0.0f); // Centre of the cube
	float cubeSize = 60.0f; // Cube extends in the y-axis, represents half the total height
	
	glm::vec3 cubeHalfExtents = glm::vec3(cubeSize) / 2.0f;  

	// Simplified drag constant for the simulation
	float k = 0.02f;

	// Bounds for walls around the top side of the cuboid
	float top  = cubeCentre.y + cubeHalfExtents.y * 10;
	float minX = cubeCentre.x - cubeHalfExtents.x * 10;
	float maxX = cubeCentre.x + cubeHalfExtents.x * 10;
	float minZ = cubeCentre.z - cubeHalfExtents.z * 10;
	float maxZ = cubeCentre.z + cubeHalfExtents.z * 10;

	


	// Iterate over each particle to update their physics state
	for (int i = 0; i < TOTAL_PARTICLES; i++) {
		Particle& p = particles[i];

		// Calculate drag force for the current particle
		vec3 velocity = p.Velocity();
		vec3 dragForce = -k * glm::length(velocity) * velocity; // Quadratic drag formula
		vec3 acceleration = GRAVITY + dragForce / p.Mass(); // Total acceleration from gravity and drag

		// Update position and velocity using symplectic Euler
		vec3 pos = p.Position(), vel = p.Velocity();
		SymplecticEuler(pos, vel, p.Mass(), acceleration, glm::vec3(0.0f), deltaTime);

		// Check and handle collision with the ground
		if (pos.y <= PARTICLE_RADIUS) {
			pos.y = PARTICLE_RADIUS; // Correct position
			if (vel.y < 0) vel.y = -vel.y * coefficientOfRestitution; // Reflect velocity
		}

		// Finalize updates
		p.SetPosition(pos);
		p.SetVelocity(vel);
		AddParticleToGrid(p); 
	}
	// Iterate over each particle to update their physics state with wall collision
	for (int i = 0; i < TOTAL_PARTICLES; i++) {
		Particle& p = particles[i];
		glm::vec3 position = p.Position();
		glm::vec3 velocity = p.Velocity();
		glm::vec3 impulse = glm::vec3(0.0f);

		// Check and handle collision with the top surface and the walls
		if (position.y >= top) { // Collision with the top surface
			position.y = top; // Correct position to be exactly at the top
			velocity.y = -velocity.y * coefficientOfRestitution;
		}

		if (position.x <= minX || position.x >= maxX) { // Collision with x-axis walls
			velocity.x = -velocity.x * coefficientOfRestitution;
			if (position.x <= minX) {
				position.x = minX;
			}
			else {
				position.x = maxX;
			}
		}

		if (position.z <= minZ || position.z >= maxZ) { // Collision with z-axis walls
			velocity.z = -velocity.z * coefficientOfRestitution;
			if (position.z <= minZ) {
				position.z = minZ;
			}
			else {
				position.z = maxZ;
			}
		}

		// Update particle's position and velocity after collision
		p.SetPosition(position);
		p.SetVelocity(velocity);
	}


	// Handle collisions within each grid cell
	for (auto& row : grid) {
		for (auto& cellColumn : row) {
			for (auto& cell : cellColumn) {
				HandleCollisionsWithinCell(cell, deltaTime);
			}
		}
	}

	
}







// This is called every frame, after Update
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{

	for (int i = 0; i < TOTAL_PARTICLES; i++) {
		particles[i].Draw(viewMatrix, projMatrix);
	}

	ground.Draw(viewMatrix, projMatrix);
}

void PhysicsEngine::HandleInputKey(int keyCode, bool pressed)
{
	if (pressed) {
		switch (keyCode) {
		case 'R': // Restart with the same seed
			Task1Init(); // Reinitialize particles with the same seed
			break;
		case 'U': // Restart with a new random seed
			useNewSeed = true; // Set flag to use a new seed
			Task1Init(); // Reinitialize particles with a new seed
			break;
			// Other cases...
		}
	}
}
