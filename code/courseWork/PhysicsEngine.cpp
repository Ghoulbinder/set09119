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
const int TOTAL_PARTICLES = 30;
Particle particles[TOTAL_PARTICLES];

const float dampingFactor = 0.95f; // Adjust as needed 
//const float coefficientOfRestitution = 0.7f; //  adjust as needed 
bool shouldPrint = true; // Set to false to disable printing


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

//void applyImpulse(Particle& particle1, Particle& particle2, const vec3& pointOfImpact, const vec3& impulse) {
//	// Calculate the r vectors from the center of mass to the point of impact for each object
//	vec3 r1 = pointOfImpact - particle1.Position();
//	vec3 r2 = pointOfImpact - particle2.Position();
//
//	// Update linear velocities
//	vec3 v1_prime = particle1.Velocity() + impulse / particle1.Mass();
//	vec3 v2_prime = particle2.Velocity() - impulse / particle2.Mass(); // Assuming impulse acts in opposite direction for object2
//
//	particle1.SetVelocity(v1_prime);
//	particle2.SetVelocity(v2_prime);
//
//	// Calculate the torque and update angular velocities
//	vec3 torque1 = cross(r1, impulse);
//	vec3 torque2 = cross(r2, -impulse); // Torque in opposite direction for the second object
//
//	// Assuming we have functions to convert torque to angular velocity change
//	vec3 angularVelocityChange1 = particle1.InverseInertia() * torque1;
//	vec3 angularVelocityChange2 = particle2.InverseInertia() * torque2;
//
//	particle1.SetAngularVelocity(particle1.AngularVelocity() + angularVelocityChange1);
//	particle2.SetAngularVelocity(object2.AngularVelocity() + angularVelocityChange2);
//}



vec3 GroundCollisionResponse(Particle& particle, float groundY, float coefficientOfRestitution) {
	glm::vec3 particlePos = particle.Position();
	glm::vec3 velocity = particle.Velocity();
	glm::vec3 impulse = glm::vec3(0.0f);
	coefficientOfRestitution = 0.1f; //  adjust as needed  
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


// Function to check collision with walls for all particles cube being used is for the ground cube
vec3 CheckCollisionWithWalls(Particle& particle, const glm::vec3& cubeCentre, float cubeSize, float coefficientOfRestitution) {
	// Extract particle properties
	glm::vec3 particlePos = particle.Position();
	glm::vec3 velocity = particle.Velocity();
	glm::vec3 impulse = glm::vec3(0.0f);
	glm::vec3 cubeHalfExtents = glm::vec3(cubeSize) / 2.0f;
	coefficientOfRestitution = 0.1f; //  adjust as needed  

	// Check for collision with cube walls
	for (int i = 0; i < 3; ++i) { // Iterate over x, y, z axes
		if (particlePos[i] < (cubeCentre[i] - cubeHalfExtents[i]) || particlePos[i] > (cubeCentre[i] + cubeHalfExtents[i])) {
			// Reflect velocity on the i-th axis
			velocity[i] = -velocity[i] * coefficientOfRestitution;
			// Calculate impulse based on change in velocity
			impulse[i] = particle.Mass() * (velocity[i] - particle.Velocity()[i]);

			// Correct position if out of bounds, to prevent sticking to walls
			if (particlePos[i] < (cubeCentre[i] - cubeHalfExtents[i])) {
				particlePos[i] = cubeCentre[i] - cubeHalfExtents[i] + 0.001f;
			}
			else if (particlePos[i] > (cubeCentre[i] + cubeHalfExtents[i])) {
				particlePos[i] = cubeCentre[i] + cubeHalfExtents[i] - 0.001f;
			}
		}
	}

	// Update particle's velocity and position
	particle.SetVelocity(velocity * dampingFactor);
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
	ground.SetScale(vec3(30.0f, 1.0f, 30.0f));
	ground.SetPosition(vec3(0.0f, 0.0f, 0.0f));//changed the y to -1 so to account for the scale of 1 so the spheres remain on 0

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
		float posX = static_cast<float>(rand()) / RAND_MAX * 30.0f - 15.0f; // Random X between -15 and 15
		float posY = static_cast<float>(rand()) / RAND_MAX * 4.0f - 2.0f;
		float posZ = static_cast<float>(rand()) / RAND_MAX * 30.0f - 15.0f; // Random Z between -15 and 15


		// Randomize velocities between -20 and 20 m/sec for the x and z axes
		//float velX = static_cast<float>(rand()) / RAND_MAX * 40.0f - 20.0f; // Random velocity X between -20 and 20
		//// Assuming no specific requirement for Y velocity, set to a default or randomize as needed
		//float velY = 0.0f; // Default Y velocity
		//float velZ = static_cast<float>(rand()) / RAND_MAX * 40.0f - 20.0f; // Random velocity Z between -20 and 20
		// Randomize velocities to be between 1 and 3 for each axis
		float velX = static_cast<float>(rand()) / RAND_MAX * 2.0f + 1.0f; // Random velocity X between 1 and 3
		float velY = static_cast<float>(rand()) / RAND_MAX * 2.0f + 1.0f; 
		float velZ = static_cast<float>(rand()) / RAND_MAX * 2.0f + 1.0f; // Random velocity Z between 1 and 3

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
	}



	camera = Camera(vec3(10, 30, 50));
}




// This is called every frame
// This should be called every fixed time step, e.g., 0.016 seconds for 60Hz
///CHECK WITH BABIS IF I SHOULD USE DELTATIME INSTEAD OF FIXEDdELTATIME IN UPDATE LOOP
void PhysicsEngine::Update(const float deltaTime) {
	// Logic that was previously inside the while loop of the accumulator
	for (int i = 0; i < TOTAL_PARTICLES; ++i) {
		Force::Gravity(particles[i]);
	}

	// Define ground and wall properties
	const float groundY = 0.0f; // Ground level
	const float coefficientOfRestitution = 0.1f; // Adjust as needed for bounce effect
	const glm::vec3 cubeCentre = glm::vec3(0.0f, groundY, 0.0f); // Centre of the cube (walls)
	const float cubeSize = 60.0f; // Total size of the cube, adjust to fit your simulation space

	// Collision detection and response
	for (int i = 0; i < TOTAL_PARTICLES; i++) {
		for (int j = i + 1; j < TOTAL_PARTICLES; j++) {
			Particle& p1 = particles[i];
			Particle& p2 = particles[j];
			vec3 delta = p2.Position() - p1.Position();
			float dist = glm::length(delta);
			float sumRadii = PARTICLE_RADIUS * 2; // Assuming all particles have the same radius

			if (dist <= sumRadii) { // Collision detected
				vec3 n = glm::normalize(delta);
				vec3 v_rel = p2.Velocity() - p1.Velocity();
				float e = std::min(p1.CoefficientOfRestitution(), p2.CoefficientOfRestitution()); // Use the smaller restitution coefficient
				float J = -(1 + e) * glm::dot(v_rel, n) / (1 / p1.Mass() + 1 / p2.Mass());

				// Update velocities
				p1.SetVelocity(p1.Velocity() + J / p1.Mass() * n * dampingFactor);
				p2.SetVelocity(p2.Velocity() - J / p2.Mass() * n * dampingFactor);
			}
		}
	}
	
	for (int i = 0; i < TOTAL_PARTICLES; i++) {
		Particle& particle = particles[i];
		GroundCollisionResponse(particle, groundY, coefficientOfRestitution);

		// Handle wall collision
		glm::vec3 impulseFromWall = CheckCollisionWithWalls(particle, cubeCentre, cubeSize, coefficientOfRestitution);
		// Optionally use impulseFromWall if you plan to apply reaction forces or for further calculations
	}

	//symplectic Euler integration
	for (int i = 0; i < TOTAL_PARTICLES; i++) {
		Particle& particle = particles[i];
		glm::vec3 pos = particle.Position(); // Temporary copy
		glm::vec3 vel = particle.Velocity() * dampingFactor; // Temporary copy
		glm::vec3 acceleration = particle.AccumulatedForce() / particle.Mass();

		// Update the temporary variables
		SymplecticEuler(pos, vel, particle.Mass(), acceleration, glm::vec3(0.0f), deltaTime);


		if (shouldPrint) {
			//Check for NaN values in velocity before updating the particle
			if (isnan(vel.x) || isnan(vel.y) || isnan(vel.z)) {
				std::cerr << "Invalid velocity detected for particle " << i << std::endl;
				// You might want to handle the error, like resetting the velocity or continuing to the next particle
				continue; // Skipping this particle for now
			}

			std::cout << "Particle " << i << " Velocity: " << glm::to_string(vel) << std::endl;
		}
		 //Update the particle with the new values
		particle.SetPosition(pos);
		particle.SetVelocity(vel);
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
		//switch (keyCode) {
		//case 'R': // Restart with the same seed
		//	Init(camera, meshDb, shaderDb);
		//	break;
		//case 'U': // Restart with a new seed
		//	srand(static_cast<unsigned int>(time(0)) + 1);
		//	Init(camera, meshDb, shaderDb);
		//	break;
		//default:
		//	break;
		//}
	}
}