#include "PhysicsEngine.h"
#include "Application.h"
#include "Camera.h"

using namespace glm;

// Define the gravity vector for the simulation
const glm::vec3 GRAVITY = glm::vec3(0, -9.81, 0);

// Implement the Explicit Euler integration method(placeholder for future use)
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

// Implement the Symplectic Euler integration method for better numerical stability
void SymplecticEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{

	// Using the integrator, compute the new velocity (vel+1)
	vel = vel + dt * accel;

	// Using the integrator, compute the new position (pos+1)
	pos = pos + dt * vel;

}
void Verlet(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Implement
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

void Rk4(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Implement
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}



// Calculate the collision when a particle collides with the cube walls without forces
vec3 CheckCollision(Particle& particle, const glm::vec3& cubeCentre, float cubeSize, float coefficientOfRestitution) {
	glm::vec3 particlePos = particle.Position();
	glm::vec3 velocity = particle.Velocity();
	glm::vec3 impulse = glm::vec3(0.0f);
	glm::vec3 cubeHalfExtents = glm::vec3(cubeSize) / 2.0f;

	// Check for collision with cube walls
	for (int i = 0; i < 3; ++i) { // Iterate over x, y, z axes
		if (particlePos[i] < (cubeCentre[i] - cubeHalfExtents[i]) || particlePos[i] > (cubeCentre[i] + cubeHalfExtents[i])) {
			// Reflect velocity on the i-th axis
			velocity[i] = -velocity[i] * coefficientOfRestitution;
			// Calculate impulse based on change in velocity
			impulse[i] = particle.Mass() * (velocity[i] - particle.Velocity()[i]);

			// Correct position if out of bounds, to prevent sticking
			if (particlePos[i] < (cubeCentre[i] - cubeHalfExtents[i])) {
				particlePos[i] = cubeCentre[i] - cubeHalfExtents[i] + 0.001f;
			}
			else if (particlePos[i] > (cubeCentre[i] + cubeHalfExtents[i])) {
				particlePos[i] = cubeCentre[i] + cubeHalfExtents[i] - 0.001f;
			}
		}
	}

	// Update particle's velocity and position
	particle.SetVelocity(velocity);
	particle.SetPosition(particlePos);

	return impulse;
}


// Placeholder function for calculating blow dryer force = wind
vec3 BlowDryerForce(const vec3& particlePosition, float cone_y_base, float cone_y_tip, float cone_r_base, float max_force = 100)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Calculate blow dryer force
	vec3 force = {0,0,0};
	return force;
}

// This is called once and it Initialises the physics engine with meshes, shaders and initial configurations
void PhysicsEngine::Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	// Get a few meshes/shaders from the databases
	auto defaultShader = shaderDb.Get("default");
	auto particleMesh = meshDb.Get("tetra");
	auto groundMesh = meshDb.Get("plane");
	auto cubeMesh = meshDb.Get("cube"); 

	meshDb.Add("cube", Mesh(MeshDataFromWavefrontObj("resources/models/cube.obj")));
	meshDb.Add("sphere", Mesh(MeshDataFromWavefrontObj("resources/models/sphere.obj")));
	meshDb.Add("cone", Mesh(MeshDataFromWavefrontObj("resources/models/cone.obj")));
	auto mesh = meshDb.Get("cube");

	//distanc ebetween particles
	float separation = 0.5f;

	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(10.0f));
	ground.SetPosition(vec3(0.0f, 0.0f, 0.0f));

	// Initialise cube
	cube.SetMesh(cubeMesh); 
	cube.SetShader(defaultShader);  
	cube.SetScale(vec3(20.0f));  // Adjust the cube to match the ground's scale
	cube.SetPosition(vec3(0.0f, 0.0f, 0.0f));  // Center the cube at the same position as the ground

	// Initialise particle
	particle.SetMesh(mesh);
	particle.SetShader(defaultShader);
	particle.SetColor(vec4(1, 0, 0, 1));
	particle.SetPosition(vec3(0, 5, 0));
	particle.SetScale(vec3(0.1f));
	particle.SetVelocity(vec3(1.f, 0.0f, 2.f));

	particle2.SetMesh(mesh);
	particle2.SetShader(defaultShader);
	particle2.SetColor(vec4(0, 1, 0, 1)); // 
	particle2.SetPosition(vec3(0 + separation, 5, 0));
	particle2.SetScale(vec3(0.1f));
	particle2.SetVelocity(vec3(1.f, 0.0f, 2.f));

	// Repeat for particle3 and particle4 with increasing separation
	particle3.SetMesh(mesh);
	particle3.SetShader(defaultShader);
	particle3.SetColor(vec4(0, 0, 1, 1));
	particle3.SetPosition(vec3(0 + 2 * separation, 5, 0));
	particle3.SetScale(vec3(0.1f));
	particle3.SetVelocity(vec3(1.f, 0.0f, 2.f));

	particle4.SetMesh(mesh);
	particle4.SetShader(defaultShader);
	particle4.SetColor(vec4(1, 1, 0, 1));
	particle4.SetPosition(vec3(0 + 3 * separation, 5, 0));
	particle4.SetScale(vec3(0.1f));
	particle4.SetVelocity(vec3(1.f, 0.0f, 2.f));

	camera = Camera(vec3(0, 2.5, 10));

}

// Update the physics simulation every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{
	float coefficientOfRestitution = 0.9f;
	vec3 cubeCentre = glm::vec3(0.0f); // Cube is centered at the origin
	float cubeSize = 10.0f; // Cube extends from -1 to 1 in all axes  


	// Simplified drag constant for the simulation
	float k = 0.02f;
	vec3 velocity = particle.Velocity();
	vec3 dragForce = -k * glm::length(velocity) * velocity; // Quadratic drag
	vec3 impulse = { 0, 0, 0 };


	// Total acceleration from gravity and drag
	vec3 acceleration = GRAVITY + dragForce / particle.Mass();

	vec3 impulse1 = CheckCollision(particle, cubeCentre, cubeSize, coefficientOfRestitution);
	vec3 impulse2 = CheckCollision(particle2, cubeCentre, cubeSize, coefficientOfRestitution);
	vec3 impulse3 = CheckCollision(particle3, cubeCentre, cubeSize, coefficientOfRestitution);
	vec3 impulse4 = CheckCollision(particle4, cubeCentre, cubeSize, coefficientOfRestitution);





	vec3 p = particle.Position(), v = particle.Velocity();
	vec3 p2 = particle2.Position(), v2 = particle2.Velocity();
	vec3 p3 = particle3.Position(), v3 = particle3.Velocity();
	vec3 p4 = particle4.Position(), v4 = particle4.Velocity();

	// Assuming impulse2 is calculated and includes all impulses acting on particle2
	

	// Apply Explicit Euler to particle2
	ExplicitEuler(p2, v2, particle2.Mass(), GRAVITY, impulse2, deltaTime);


	SymplecticEuler(p , v , particle.Mass(), acceleration, impulse, deltaTime);
	
	
	particle.SetPosition(p);
	particle.SetVelocity(v);

	particle2.SetPosition(p2); 
	particle2.SetVelocity(v2); 

	particle3.SetPosition(p3); 
	particle3.SetVelocity(v3);

	particle4.SetPosition(p4); 
	particle4.SetVelocity(v4); 

	// Adjust positions and velocities as necessary
	particle2.SetPosition(particle2.Position()); 
	particle2.SetVelocity(particle2.Velocity()); 

	particle3.SetPosition(particle3.Position()); 
	particle3.SetVelocity(particle3.Velocity()); 

	particle4.SetPosition(particle4.Position()); 
	particle4.SetVelocity(particle4.Velocity()); 


	// Array of pointers to each particle
	Particle* particles[4] = { &particle, &particle2, &particle3, &particle4 }; 

	// Iterate over each particle to check for collision with the ground
	for (int i = 0; i < 4; ++i) { 
		Particle* p = particles[i]; // Get pointer to current particle 
		


		 
		if (p->Position().y <= 0.0f) {  
			// Particle is colliding with the ground
			vec3 particleVelocity = p->Velocity();  

			// Calculate the reflected velocity using the law of reflection
			// Assuming the ground is horizontal, so only the vertical component is inverted
			particleVelocity.y = -particleVelocity.y * 0.9f; // Adjust coefficientOfRestitution as needed 

			// Update the particle's velocity with the reflected velocity
			p->SetVelocity(particleVelocity);  

			// Move the particle slightly above the ground to prevent sticking
			p->SetPosition(vec3(p->Position().x, 0.001f, p->Position().z));  
		}
	}


}

// This is called every frame, after Update and Renders the simulation
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	particle.Draw(viewMatrix, projMatrix);
	particle2.Draw(viewMatrix, projMatrix);  
	particle3.Draw(viewMatrix, projMatrix); 
	particle4.Draw(viewMatrix, projMatrix); 
	ground.Draw(viewMatrix, projMatrix);
}

// Handles keyboard input
void PhysicsEngine::HandleInputKey(int keyCode, bool pressed)
{
	switch (keyCode)
	{
	case GLFW_KEY_1:
		printf("Key 1 was %s\n", pressed ? "pressed" : "released");
		break; // don't forget this at the end of every "case" statement!
	default:
		break;
	}
}