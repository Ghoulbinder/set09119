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


// Adjusted for 3 grids of 10x10 particles each
const int TOTAL_PARTICLES = 1000;
RigidBody particles[TOTAL_PARTICLES]; 

const float dampingFactor = 0.95f; // Adjust as needed 
//const float coefficientOfRestitution = 0.7f; //  adjust as needed 
bool shouldPrint = false; // Set to false to disable printing

float coefficientOfRestitution = 0.9f; // Bounciness of the cube's walls 

std::vector<glm::vec4> colorOptions = {
	glm::vec4(1.0f, 0.0f, 0.0f, 1.0f),  // Red
	glm::vec4(0.0f, 1.0f, 0.0f, 1.0f),  // Green
	glm::vec4(0.0f, 0.0f, 1.0f, 1.0f)   // Blue
}; 


void UpdateAngularIntegration(RigidBody& rb, float deltaTime) {
	auto newAngVel = rb.AngularVelocity() + deltaTime * rb.AngularAcceleration();
	rb.SetAngularVelocity(newAngVel);

	mat3 angVelSkew = matrixCross3(newAngVel);
	mat3 R = mat3(rb.Orientation());
	R += deltaTime * angVelSkew * R;
	R = glm::orthonormalize(R);

	rb.SetOrientation(glm::mat4(R));
}
 
void SymplecticEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt, RigidBody& rb) {
	// Using the integrator, compute the new velocity (vel+1)
	vel = vel + dt * accel;

	// Using the integrator, compute the new position (pos+1)
	pos = pos + dt * vel;

	// Update angular velocity and orientation
	UpdateAngularIntegration(rb, dt);
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

	

	srand(static_cast<unsigned int>(time(0)));
	// Store the current time as the seed
	initialSeed = static_cast<unsigned int>(std::time(nullptr));   
	// Sphere type definitions
	vec4 colors[] = { vec4(1, 0, 0, 1), vec4(0, 1, 0, 1), vec4(0, 0, 1, 1) };
	float masses[] = { 1.0f, 2.0f, 3.0f };


	int numParticlesPerGrid = 1; // Adjust as needed
	
	float initialY = PARTICLE_RADIUS;


	// Initialize the seed only if it has not been initialized before
	if (initialSeed == 0) {
		initialSeed = static_cast<unsigned int>(std::time(nullptr));
		
	}

		
	generator.seed(initialSeed);
	
	
	
	std::uniform_real_distribution<float> positionDistXZ(-150.0f, 150.0f);
	std::uniform_real_distribution<float> velocityDistXZ(-20.0f, 20.0f);  // Velocity distribution for x and z
	std::uniform_real_distribution<float> angleDist(-1.0f, 1.0f);  // Angular velocity range 
	std::uniform_int_distribution<> colorIndexDist(0, 2); // Random index for selecting color
	std::vector<float> massOptions = { 1.0f, 2.0f, 3.0f };  // Possible masses  
	std::uniform_int_distribution<> massIndexDist(0, massOptions.size() - 1); 


	for (int i = 0; i < TOTAL_PARTICLES; ++i) {
		glm::vec3 pos(positionDistXZ(generator), 0.0f, positionDistXZ(generator));
		glm::vec3 vel(velocityDistXZ(generator), 0.0f, velocityDistXZ(generator));
		glm::vec3 angVel(angleDist(generator), angleDist(generator), angleDist(generator));  // Generate random angular velocity 
		float mass = massOptions[massIndexDist(generator)];  // Select a mass randomly from the options
		glm::vec4 color = colorOptions[colorIndexDist(generator)]; // Select color randomly

		particles[i].SetPosition(pos);
		particles[i].SetVelocity(vel);
		particles[i].SetAngularVelocity(angVel);  // Set the random angular velocity 
		particles[i].SetMass(mass);  // Set the randomly chosen mass
		// Set other properties like mesh and shader, etc.
		particles[i].SetMesh(meshDb.Get("sphere"));
		particles[i].SetShader(shaderDb.Get("default"));
		particles[i].SetColor(color);  // Set the randomly chosen color 
		//particles[i].SetColor(vec4(rand() % 256 / 255.0f, rand() % 256 / 255.0f, rand() % 256 / 255.0f, 1.0));
	

		// Add particle to grid
		AddParticleToGrid(particles[i]); 
	}


	camera = Camera(vec3(10, 100, 100)); 
} 


void PhysicsEngine::Task1Init() {
	std::uniform_real_distribution<float> positionDistXZ(-150.0f, 150.0f);
	std::uniform_real_distribution<float> velocityDistXZ(-20.0f, 20.0f);
	std::vector<float> massOptions = { 1.0f, 2.0f, 3.0f };
	std::uniform_int_distribution<> massIndexDist(0, massOptions.size() - 1); // Same mass distribution as in Init
	std::uniform_int_distribution<> colorIndexDist(0, 2);

	for (Particle& particle : particles) {
		glm::vec3 pos(positionDistXZ(generator), 0.0f, positionDistXZ(generator));
		glm::vec3 vel(velocityDistXZ(generator), 0.0f, velocityDistXZ(generator));
		float mass = massOptions[massIndexDist(generator)];  // Randomly select mass
		glm::vec4 color = colorOptions[colorIndexDist(generator)]; // Select color randomly 

		particle.SetPosition(pos);
		particle.SetVelocity(vel);
		particle.SetMass(mass);
		particle.SetColor(color); 
		AddParticleToGrid(particle);
	}

}


void PhysicsEngine::Task1Update(float deltaTime, float totalTime){
	



}


// This is called every frame
// This should be called every fixed time step, e.g., 0.016 seconds for 60Hz
///CHECK WITH BABIS IF I SHOULD USE DELTATIME INSTEAD OF FIXEDdELTATIME IN UPDATE LOOP
void PhysicsEngine::Update(const float deltaTime) {
	ClearGrid(); // Clear the grid for new assignments 
	

	// Define properties of the environment
	const glm::vec3 cubeCentre = glm::vec3(0.0f, 0.0f, 0.0f); // Centre of the cube
	float cubeSize = 60.0f; // Cube extends in the y-axis, represents half the total height
	
	glm::vec3 cubeHalfExtents = glm::vec3(cubeSize) / 2.0f;  

	// Simplified drag constant for the simulation
	// Use drag constant based on toggle  
	float k = dragEnabled ? 0.02f : 0.0f; 
	
	// Bounds for walls around the top side of the cuboid
	float top  = cubeCentre.y + cubeHalfExtents.y * 10;
	float minX = cubeCentre.x - cubeHalfExtents.x * 10;
	float maxX = cubeCentre.x + cubeHalfExtents.x * 10;
	float minZ = cubeCentre.z - cubeHalfExtents.z * 10;
	float maxZ = cubeCentre.z + cubeHalfExtents.z * 10;



	// Iterate over each particle to update their physics state
	for (int i = 0; i < TOTAL_PARTICLES; i++) {
		RigidBody& rb = particles[i]; 

		// Calculate drag force for the current particle
		vec3 velocity = rb.Velocity();
		vec3 dragForce = -k * glm::length(velocity) * velocity; // Quadratic drag formula
		glm::vec3 acceleration = GRAVITY + (dragEnabled ? dragForce : glm::vec3(0)) / rb.Mass(); 


		// Update position and velocity using symplectic Euler
		vec3 pos = rb.Position(), vel = rb.Velocity(); 
		SymplecticEuler(pos, vel, rb.Mass(), acceleration, glm::vec3(0.0f), deltaTime, rb); 

		// Update angular properties using provided function
		UpdateAngularIntegration(rb, deltaTime); 

		// Check and handle collision with the ground
		if (pos.y <= PARTICLE_RADIUS) {
			pos.y = PARTICLE_RADIUS; // Correct position
			if (vel.y < 0) vel.y = -vel.y * coefficientOfRestitution; // Reflect velocity
		}

		// Finalize updates
		rb.SetPosition(pos);
		rb.SetVelocity(vel);


		AddParticleToGrid(rb);
	}

	// Iterate over each particle to update their physics state with wall collision
	for (int i = 0; i < TOTAL_PARTICLES; i++) {
		RigidBody& rb = particles[i];  
		glm::vec3 position = rb.Position(); 
		glm::vec3 velocity = rb.Velocity(); 
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
		rb.SetPosition(position); 
		rb.SetVelocity(velocity); 
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
			// Reinitialize particles with the same seed
			generator.seed(initialSeed); 
			Task1Init(); // Reinitialize particles with the same seed
			break;
		case 'U': // Restart with a new random seed
			// Generate a new seed, reseed the generator, and reinitialize particles
			initialSeed = static_cast<unsigned int>(std::time(nullptr));  // Update the initial seed with a new value
			generator.seed(initialSeed); 
			Task1Init(); // Reinitialize particles with a new seed
			break;
		case 'E':
			dragEnabled = !dragEnabled;  // Toggle the drag force on or off
			std::cout << "Drag force is now " << (dragEnabled ? "enabled" : "disabled") << ".\n";
			break;
			// Other cases...
		}
	}
}
