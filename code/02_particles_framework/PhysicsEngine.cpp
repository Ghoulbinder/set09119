#include "PhysicsEngine.h"
#include "Application.h"
#include "Camera.h"

using namespace glm;

// Define the gravity vector for the simulation
const glm::vec3 GRAVITY = glm::vec3(0, -9.81, 0);

// Implement the Explicit Euler integration method(placeholder for future use)
void ExplicitEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Implement
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

// Implement the Symplectic Euler integration method for better numerical stability
void SymplecticEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	
	// Using the integrator, compute the new velocity (vel+1)
	vel = vel + dt * accel;

	// Using the integrator, compute the new position (pos+1)
	pos = pos + dt * vel;
	
}

// Calculate the collision impulse when a particle collides with the cube walls
vec3 CollisionImpulse(Particle& pobj, const glm::vec3& cubeCentre, float cubeSize, float coefficientOfRestitution = 0.9f) {
	glm::vec3 cubeHalfExtents = glm::vec3(cubeSize) / 2.0f;
	glm::vec3 particlePos = pobj.Position();
	glm::vec3 velocity = pobj.Velocity();
	vec3 impulse(0.0f);

	// check for collisions with the cube walls
	for (int i = 0; i < 3; ++i) {
		float distanceFromCenter = abs(particlePos[i] - cubeCentre[i]);
		float maxExtent = cubeHalfExtents[i];

		if (distanceFromCenter > maxExtent) {
			// Calculate normal based on which side of the cube the collision occurred
			vec3 normal(0.0f);
			
			if (particlePos[i] > cubeCentre[i]) { 
				normal[i] = 1.0f; 
			}
			else {
				normal[i] = -1.0f; 
			}

			// Reflect velocity
			velocity -= 2 * dot(velocity, normal) * normal * coefficientOfRestitution;

			// Adjust position to prevent sticking. This part may need tuning.
			if (particlePos[i] > cubeCentre[i]) {
				particlePos[i] = cubeCentre[i] + maxExtent + 0.001f; // Move outside the cube slightly
			}
			else {
				particlePos[i] = cubeCentre[i] - maxExtent - 0.001f;
			}
		}
	}

	// Apply updated velocity and position
	pobj.SetVelocity(velocity);
	pobj.SetPosition(particlePos);

	return impulse; // The actual 'impulse' vector might not be directly used in this context
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

	camera = Camera(vec3(0, 2.5, 10));

}

// Update the physics simulation every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{
	float coefficientOfRestitution = 0.9f;
	vec3 cubeCentre = glm::vec3(0.0f); // Cube is centered at the origin
	float cubeSize = 10.0f; // Cube extends from -1 to 1 in all axes  
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Handle collisions and calculate impulse
	// Basic aerodynamic drag parameters
	//Fdrag?=?1/2(??v^2Cd?Av) 
	// ? is the density of the fluid (air, in this case).
	//v is the velocity of the object relative to the fluid
	//	Cd is the drag coefficient, which depends on the shape of the object and the roughness of its surface.
	//	A is the cross - sectional area of the object perpendicular to the velocity vector.
	//	v is the unit vector in the direction of the velocity.
	// Fdrag = ?k?v^2v Where k is a constant that encapsulates the drag coefficient, 
	// cross - sectional area, and fluid density, adjusted for the simulation's scale and units.

	// Simplified drag constant for the simulation
	float k = 0.02f;
	vec3 velocity = particle.Velocity();
	vec3 dragForce = -k * glm::length(velocity) * velocity; // Quadratic drag

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	//acceleration calculation
	//vec3 acceleration = GRAVITY; 

	// Total acceleration from gravity and drag
	vec3 acceleration = GRAVITY + dragForce / particle.Mass();

	vec3 impulse = CollisionImpulse(particle, cubeCentre, cubeSize, coefficientOfRestitution);


	// Calculate acceleration by accumulating all forces (here we just have gravity) and dividing by the mass
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Implement a simple integration scheme
	vec3 p = particle.Position(), v = particle.Velocity();

	SymplecticEuler(p, v, particle.Mass(), acceleration, impulse, deltaTime);
	particle.SetPosition(p);
	particle.SetVelocity(v);

	// Check for collision with the ground
	if (particle.Position().y <= 0.0f) {
		// Particle is colliding with the ground
		vec3 particleVelocity = particle.Velocity();

		// Calculate the reflected velocity using the law of reflection
		// Assuming the ground is horizontal, so only the vertical component is inverted
		particleVelocity.y = -particleVelocity.y * 0.9f; // Adjust coefficientOfRestitution as needed

		// Update the particle's velocity with the reflected velocity
		particle.SetVelocity(particleVelocity);

		// Move the particle slightly above the ground to prevent sticking
		particle.SetPosition(vec3(particle.Position().x, 0.001f, particle.Position().z));
	}


}

// This is called every frame, after Update and Renders the simulation
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	particle.Draw(viewMatrix, projMatrix);
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