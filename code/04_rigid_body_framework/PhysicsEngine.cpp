#include "PhysicsEngine.h"

#include <map>
#include <numeric>

#include "Application.h"
#include "Camera.h"
#include "Force.h"
#include <string>
#include <iostream>

#include <glm/gtx/matrix_cross_product.hpp>
#include <glm/gtx/orthonormalize.hpp>


using namespace glm;
using namespace std;

const vec3 GRAVITY = vec3(0, -9.81, 0);

const float GROUND_Y = 0.0f; // since the ground is at y = 0.0f

RigidBody rbody[1];

int scenario = 0; // This keeps track of the current scenario

// Define flags for each scenario to track whether the impulse has already been applied.
bool scenarioApplied[4] = { false }; // Initialize all scenario flags as false. Adjust the size based on the number of scenarios.
bool forceApplied[4] = { false }; // Adjust the size based on the number of scenarios.
static double scenarioTime[4] = { 0.0 }; // Time since last force application for each scenario
 
//DO NOT SET VELOCITY AND POSISTION TO 0 WHEN CALCULATING THE IMMPULSE


// A helper function to convert a vec3 to a string
std::string vec3_to_string(const glm::vec3& vec) {
	return "(" + std::to_string(vec.x) + ", " + std::to_string(vec.y) + ", " + std::to_string(vec.z) + ")";
}


void SymplecticEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	vel = vel + impulse / mass;

	vel = vel + dt * accel;

	// Using the integrator, compute the new position (pos+1)
	pos = pos + dt * vel;
}


vec3 CalculateContactPoint(const RigidBody& rb, const vec3& planeNormal, float planeDistance)
{
	// Calculate the distance from the center of the rigid body to the ground plane
	float distanceToPlane = glm::dot(rb.Position(), planeNormal) - planeDistance;

	// Calculate the contact point by projecting the center of the rigid body onto the ground plane
	vec3 contactPoint = rb.Position() - distanceToPlane * planeNormal;
	return contactPoint;
}

// Updated CheckCollision to consider vertices
// Checks for collision and adjusts the position and velocity of the rigid body.
void CheckAndRespondCollision(RigidBody& rb, const vec3& groundNormal, float groundY, float coefficientOfRestitution, float deltaTime) {
	bool collisionDetected = false;
	vec3 collisionPoint;
	float minPenetrationDistance = std::numeric_limits<float>::infinity();

	const auto& meshData = rb.GetMesh()->Data();
	const auto& modelMatrix = rb.ModelMatrix();

	// Loop through vertices to find the one with minimum distance to the ground (deepest penetration or closest approach)
	for (const auto& vertex : meshData.positions.data) {
		vec4 worldVertex = modelMatrix * vec4(vertex, 1.0f);
		float distanceToGround = dot(vec3(worldVertex) - vec3(0, groundY, 0), groundNormal);

		if (distanceToGround < minPenetrationDistance) {
			minPenetrationDistance = distanceToGround;
			collisionPoint = vec3(worldVertex);
			collisionDetected = distanceToGround < 0; // Considered in collision if below ground
		}
	}

	// If collision is detected or a vertex is below ground, adjust position and velocity
	if (collisionDetected) {
		// Push vertices above ground
		if (minPenetrationDistance < 0) {
			rb.SetPosition(rb.Position() + vec3(0, -minPenetrationDistance, 0));
		}

		// Reflect velocity for bounce, modifying only if moving towards ground
		if (rb.Velocity().y < 0) {
			vec3 velocity = rb.Velocity();
			velocity.y = -velocity.y * coefficientOfRestitution;
			rb.SetVelocity(velocity);
		}

		cout << "Collision detected and handled. New position: " << vec3_to_string(rb.Position()) << "\n";
	}
}




void UpdateAngularIntegration(RigidBody& rb, float deltaTime) {
	auto newAngVel = rb.AngularVelocity() + deltaTime * rb.AngularAcceleration();
	rb.SetAngularVelocity(newAngVel);

	mat3 angVelSkew = matrixCross3(newAngVel);
	mat3 R = mat3(rb.Orientation());
	R += deltaTime * angVelSkew * R;
	R = glm::orthonormalize(R);

	rb.SetOrientation(glm::mat4(R));
	cout << "Angular Integration: New angular velocity set to " << vec3_to_string(newAngVel) << endl;
}

void PerformFixedTimeStepIntegration(float deltaTime) {
	for (int i = 0; i < 1; ++i) {
		rbody[i].ClearForcesImpulses();
		CheckAndRespondCollision(rbody[i], vec3(0, 1, 0), GROUND_Y, 0.9f, deltaTime);
	}
}


void SymplecticEulerIntegration(RigidBody& rb, float deltaTime) {
	// Get the current state of the particle
	vec3 p = rb.Position();
	vec3 v = rb.Velocity();
	vec3 acceleration = rb.AccumulatedForce() / rb.Mass();

	// Use the Symplectic Euler to update velocity and position
	SymplecticEuler(p, v, rb.Mass(), acceleration, vec3(0.0f), deltaTime);

	// Apply the updated state back to the particle
	rb.SetPosition(p); 
	rb.SetVelocity(v); 

	cout << "Symplectic Euler Integration: Position updated to " << vec3_to_string(p)
		<< " and velocity updated to " << vec3_to_string(v) << endl;
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
	float separation = 0.5f;

	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(10.0f));
	ground.SetPosition(vec3(0.0f, 0.0f, 0.0f));

	// Initialize two particles
	rbody[0].SetMesh(mesh);
	rbody[0].SetShader(defaultShader);
	rbody[0].SetColor(vec4(1, 0, 0, 1));
	rbody[0].SetPosition(vec3(0.0f, 5.0f, 0.0f));
	rbody[0].SetScale(vec3(1, 3, 1));
	rbody[0].SetVelocity(vec3(0.0f, 0.0f, 0.0f));
	rbody[0].SetMass(1.0f);
	rbody[0].SetAngularVelocity(glm::vec3(0.0f, 0.0f, 0.0f)); // Example angular velocity 

	// TODO: Get the mesh and shader for rigidy body
	camera = Camera(vec3(0, 5, 10));
}


// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{
	static double accumulator = 0.0;
	const double fixedDeltaTime = 0.016; // 16ms for a 60Hz update rate 
	float coefficientOfRestitution = 0.9f;
	float frictionCoefficient = 0.5f; // Example coefficient
	vec3 impulse = rbody[0].Mass() * GRAVITY * deltaTime;
	vec3 contactPoint = rbody[0].Position() -vec3(0, rbody[0].Scale().y / 2, 0); 

	accumulator += deltaTime;

	while (accumulator >= fixedDeltaTime) {

		vec3 gravityVelocityChange = GRAVITY;
		// Directly set a constant velocity and angular velocity for the first rigid body
		rbody[0].SetVelocity(vec3(1, 0, 0)); // Apply a constant velocity of 3 units/s upwards
		rbody[0].SetAngularVelocity(vec3(0, 0, 1)); // Apply a constant angular rotation around the Z-axis
		


		// Loop through all rigid bodies and apply the physics simulation steps
		for (int i = 0; i < 1; ++i) {
			Force::Gravity(rbody[i]);  
			
			SymplecticEulerIntegration(rbody[i], fixedDeltaTime); // Integrate motion equations
			UpdateAngularIntegration(rbody[i], fixedDeltaTime); 
			CheckAndRespondCollision(rbody[0], vec3(0, 1, 0), GROUND_Y, coefficientOfRestitution, deltaTime);
			rbody[i].ApplyFriction(impulse, contactPoint, frictionCoefficient, deltaTime);
		}

		accumulator -= fixedDeltaTime; // Decrease accumulated time
	}

	
}
			


// This is called every frame, after Update
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	for (int i = 0; i < 1; i++) {
		rbody[i].Draw(viewMatrix, projMatrix);
	}

	ground.Draw(viewMatrix, projMatrix);
}


// This is called every frame, after Update
void PhysicsEngine::HandleInputKey(int keyCode, bool pressed)
{
	//if (!pressed) return; // Only handle keydown events
	
	// Determine if the scenario changes based on the keyCode,
	 //and reset all flags to false if it does.
	//int prevScenario = scenario;
	//switch (keyCode) {
	//case '0': scenario = 0; break;
	//case '1': scenario = 1; break;
	//case '2': scenario = 2; break;
	//case '3': scenario = 3; break;
	//case '4': scenario = 4; break;
	
		// Add more cases as needed...
	//}

	// //If the scenario has changed, reset all applied flags
	//if (prevScenario != scenario) { 
	//	fill(begin(scenarioApplied), end(scenarioApplied), false); // Reset all flags 
	//	fill(begin(forceApplied), end(forceApplied), false); // Reset force application flags 
	//
	//}
}


