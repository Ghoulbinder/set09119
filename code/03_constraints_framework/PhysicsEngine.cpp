#include "PhysicsEngine.h"
#include "Application.h"
#include "Camera.h"
#include "Force.h"

using namespace glm;

const glm::vec3 GRAVITY = glm::vec3(0, -9.81, 0);

// PARTICLE_RADIUS 
const float PARTICLE_RADIUS = 0.1f; // Adjustable as needed

const int GRID_SIZE = 10; 
Particle particles[GRID_SIZE * GRID_SIZE]; 

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

// Example wind force function
void ApplyWindForce(Particle& particle, const glm::vec3& windDirection, float windStrength) {
	// Apply wind force only if the particle is not fixed
	if (!particle.IsFixed()) {
		particle.ApplyForce(windDirection * windStrength);
	}
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
// This is called once
void PhysicsEngine::Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	// Get a few meshes/shaders from the databases
	auto defaultShader = shaderDb.Get("default");
	auto groundMesh = meshDb.Get("plane");
	auto cubeMesh = meshDb.Get("cube");


	meshDb.Add("cube", Mesh(MeshDataFromWavefrontObj("resources/models/cube.obj")));
	meshDb.Add("sphere", Mesh(MeshDataFromWavefrontObj("resources/models/sphere.obj")));
	meshDb.Add("cone", Mesh(MeshDataFromWavefrontObj("resources/models/cone.obj")));
	auto mesh = meshDb.Get("cube");

	//distanc ebetween particles
	//float separation = 0.5f;

	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(10.0f));
	ground.SetPosition(vec3(4.0f, -10.0f, 5.0f));

	// Initialise cube
	cube.SetMesh(cubeMesh);
	cube.SetShader(defaultShader);
	cube.SetScale(vec3(20.0f));  // Adjust the cube to match the ground's scale
	cube.SetPosition(vec3(0.0f, 0.0f, 0.0f));  // Center the cube at the same position as the ground

	float separation = 1.0f;  // Separation between particles in the grid

	float initialHeight = 5.0f; // Set all particles to this height to make the grid horizontal

	for (int y = 0; y < GRID_SIZE; ++y) {
		for (int x = 0; x < GRID_SIZE; ++x) {
			int index = y * GRID_SIZE + x;
			particles[index].SetMesh(mesh);
			particles[index].SetShader(defaultShader);
			particles[index].SetColor(vec4(1, 0, 0, 1));  // Example color, adjust as needed
			particles[index].SetScale(vec3(PARTICLE_RADIUS));

			// Adjusting Y position to initialHeight to make the grid horizontal
			particles[index].SetPosition(vec3(x * separation, initialHeight, y * separation)); // Spread out the particles along X and Z axes

			particles[index].SetVelocity(vec3(0.0f, 0.0f, 0.0f));

			// Make corner particles stationary if needed, depending on your simulation requirements
			if ((x == 0 || x == GRID_SIZE - 1) && (y == 0 || y == GRID_SIZE - 1)) {
				particles[index].SetMass(FLT_MAX);
			}
			else {
				particles[index].SetMass(1.0f);
			}
		}
	}


	camera = Camera(vec3(5, 10, 20));
}

void PhysicsEngine::Task1Init()
{
	// Clear any existing forces and impulses
	for (int i = 0; i < 10; ++i) {
		particles[i].ClearForcesImpulses();
	}

	// Set mass for all particles
	// The first particle is stationary with infinite mass
	particles[0].SetMass(FLT_MAX);
	particles[10].SetMass(FLT_MAX); // Making the 11th particle also stationary 
	//float separation = 1.0f;  

	// Normal mass for other particles
	for (int i = 1; i < 10; ++i) {
		particles[i].SetMass(1.0f);
	}

	// Set initial velocity to zero for all particles
	for (int i = 0; i < 10; ++i) {
		particles[i].SetVelocity(vec3(0.0f, 0.0f, 0.0f));
	}

	
}


void PhysicsEngine::Task1Update(float deltaTime, float totalTime)
{

	// Define wind properties
	glm::vec3 windDirection = glm::vec3(0.0f, 15.0f, 0.0f); // Wind blowing upwards 
	float windStrength = 20.0f; // Adjust the strength as needed


	// Clear forces and impulses for all particles 
	for (int i = 0; i < GRID_SIZE * GRID_SIZE; ++i) {
		particles[i].ClearForcesImpulses();
	}



	// Apply Hooke's law between each consecutive pair of particles
	float restLength = 1.0f; // Desired separation distance
	float ks = 50.0f; // Spring constant
	float kd = 0.1f; // Damping constant
	// Apply forces
	for (int y = 0; y < GRID_SIZE; ++y) {
		for (int x = 0; x < GRID_SIZE; ++x) {
			int index = y * GRID_SIZE + x;
			// Apply gravity to all but corner particles
			if (!(x == 0 || x == GRID_SIZE - 1) || !(y == 0 || y == GRID_SIZE - 1)) {
				Force::Gravity(particles[index]);
			}

			// Apply wind force to all particles
			ApplyWindForce(particles[index], windDirection, windStrength); 

			// Spring forces with adjacent particles
			if (x > 0) Force::Hooke(particles[index], particles[index - 1], restLength, ks, kd); // Left
			if (x < GRID_SIZE - 1) Force::Hooke(particles[index], particles[index + 1], restLength, ks, kd); // Right
			if (y > 0) Force::Hooke(particles[index], particles[index - GRID_SIZE], restLength, ks, kd); // Up
			if (y < GRID_SIZE - 1) Force::Hooke(particles[index], particles[index + GRID_SIZE], restLength, ks, kd); // Down
		}
	}
	for (int i = 0; i < GRID_SIZE * GRID_SIZE; ++i) {
		if (particles[i].Mass() != FLT_MAX) {
			vec3 force = particles[i].AccumulatedForce();
			vec3 impulse = particles[i].AccumulatedImpulse();
			vec3 acceleration = force / particles[i].Mass();

			vec3 newPos = particles[i].Position(); // Temporary position
			vec3 newVel = particles[i].Velocity(); // Temporary velocity

			// Now update newPos and newVel directly
			SymplecticEuler(newPos, newVel, particles[i].Mass(), acceleration, impulse, deltaTime);

			particles[i].SetPosition(newPos); // Update the particle's position
			particles[i].SetVelocity(newVel); // Update the particle's velocity
		}
	}

}

float restLength = 1.0f; // The natural length of the spring
float ks = 50.0f; // Spring constant
float kd = 0.1f; // Damping constant

// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime) {
	static double accumulator = 0.0; 
	const double fixedDeltaTime = 0.016; // 16ms for a 60Hz update rate 
	float restLength = 1.0f; // The natural length of the spring 
	float ks = 50.0f; // Spring constant 
	float kd = 0.1f; // Damping constant 


	accumulator += deltaTime;

	while (accumulator >= fixedDeltaTime) {
		// Clear forces and impulses for all particles
		for (int i = 0; i < GRID_SIZE * GRID_SIZE; ++i) {
			particles[i].ClearForcesImpulses();
		}

		// Apply gravity and spring forces
		for (int y = 0; y < GRID_SIZE; ++y) {
			for (int x = 0; x < GRID_SIZE; ++x) {
				int index = y * GRID_SIZE + x;

				// Apply gravity if not a corner particle
				if (!(x == 0 && y == 0) && !(x == GRID_SIZE - 1 && y == 0) &&
					!(x == 0 && y == GRID_SIZE - 1) && !(x == GRID_SIZE - 1 && y == GRID_SIZE - 1)) {
					Force::Gravity(particles[index]);
				}

				// Apply Hooke's law for springs to adjacent particles
				if (x < GRID_SIZE - 1) { // right
					Force::Hooke(particles[index], particles[index + 1], 1.0f, 50.0f, 0.1f);
				}
				if (y < GRID_SIZE - 1) { // below
					Force::Hooke(particles[index], particles[index + GRID_SIZE], 1.0f, 50.0f, 0.1f);
				}
			}
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

	// Example pseudo-code for drawing lines between particles:
	/*
	for (int y = 0; y < 10; y++) {
		for (int x = 0; x < 10; x++) {
			if (x < 9) { // Draw line to the right neighbor
				DrawLine(particles[y * 10 + x].Position(), particles[y * 10 + x + 1].Position(), viewMatrix, projMatrix);
			}
			if (y < 9) { // Draw line to the bottom neighbor
				DrawLine(particles[y * 10 + x].Position(), particles[(y + 1) * 10 + x].Position(), viewMatrix, projMatrix);
			}
		}
	}
	*/
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