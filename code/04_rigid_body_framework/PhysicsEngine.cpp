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
bool shouldPrint = false; // Set to false to disable printing
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

void CheckAndRespondCollision(RigidBody& rb, const vec3& groundNormal, float groundY, float coefficientOfRestitution, float deltaTime) {
	bool collisionDetected = false;
	vec3 collisionPoint;
	float minPenetrationDistance = std::numeric_limits<float>::infinity();
	vec3 averageContactPoint = vec3(0);
	int contactPoints = 0;

	const auto& meshData = rb.GetMesh()->Data();
	const auto& modelMatrix = rb.ModelMatrix();

	// Loop through vertices to find those that are below the ground
	for (const auto& vertex : meshData.positions.data) {
		vec4 worldVertex = modelMatrix * vec4(vertex, 1.0f);
		float distanceToGround = dot(vec3(worldVertex) - vec3(0, groundY, 0), groundNormal);

		if (distanceToGround < 0) { // Vertex is below ground
			collisionDetected = true;
			averageContactPoint += vec3(worldVertex);
			contactPoints++;

			if (distanceToGround < minPenetrationDistance) {
				minPenetrationDistance = distanceToGround;
				collisionPoint = vec3(worldVertex);
			}
		}
	}

	if (collisionDetected) {
		// Compute the average contact point for vertices below the ground
		averageContactPoint /= contactPoints;

		// Adjust position based on the deepest penetration point
		rb.SetPosition(rb.Position() + vec3(0, -minPenetrationDistance, 0));

		// Reflect velocity for bounce
		if (rb.Velocity().y < 0) {
			vec3 velocity = rb.Velocity();
			velocity.y = -velocity.y * coefficientOfRestitution;
			rb.SetVelocity(velocity);
		}

		// Adjust orientation based on the average contact point
		// This is a simplified approach to simulate the effect of rotation due to uneven ground contact
		// Note: This part is quite basic and might need more sophisticated logic depending on your requirements
		vec3 contactOffset = averageContactPoint - rb.Position();
		float angleAdjustment = length(contactOffset) * 0.05f; // Simplified rotational effect
		vec3 angularAdjustment = normalize(cross(contactOffset, groundNormal)) * angleAdjustment;
		rb.SetAngularVelocity(rb.AngularVelocity() + angularAdjustment);


		if (shouldPrint) {
			cout << "Collision detected and handled. New position: " << vec3_to_string(rb.Position()) << ", New Angular Velocity: " << vec3_to_string(rb.AngularVelocity()) << "\n";
		}
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
	if (shouldPrint) {
		cout << "Angular Integration: New angular velocity set to " << vec3_to_string(newAngVel) << endl;
	}
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

	if (shouldPrint) {
		cout << "Symplectic Euler Integration: Position updated to " << vec3_to_string(p)
			<< " and velocity updated to " << vec3_to_string(v) << endl;
	}
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
	//rbody[0].SetVelocity(vec3(0.0f, 0.0f, 0.0f));
	rbody[0].SetMass(1.0f);
	//rbody[0].SetAngularVelocity(glm::vec3(0.0f, 0.0f, 0.0f)); // Example angular velocity 

	//vec3 gravityVelocityChange = GRAVITY;
		// Directly set a constant velocity and angular velocity for the first rigid body
	rbody[0].SetVelocity(vec3(1, 0, 0)); // Apply a constant velocity of 3 units/s upwards
	rbody[0].SetAngularVelocity(vec3(0, 0, 1)); // Apply a constant angular rotation around the Z-axis


	// TODO: Get the mesh and shader for rigidy body
	camera = Camera(vec3(0, 5, 15));
}


// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{
	static double accumulator = 0.0;
	const double fixedDeltaTime = 0.016; // 16ms for a 60Hz update rate 
	float coefficientOfRestitution = 0.1f;
	float frictionCoefficient = 0.5f; // Example coefficient
	vec3 impulse = rbody[0].Mass() * GRAVITY * deltaTime;
	vec3 contactPoint = rbody[0].Position() -vec3(0, rbody[0].Scale().y / 2, 0); 

	accumulator += deltaTime;

	while (accumulator >= fixedDeltaTime) {

		
		// Loop through all rigid bodies and apply the physics simulation steps
		for (int i = 0; i < 1; ++i) {
			rbody[i].ClearForcesImpulses();

			Force::Gravity(rbody[i]);  
			
			SymplecticEulerIntegration(rbody[i], fixedDeltaTime); // Integrate motion equations
			UpdateAngularIntegration(rbody[i], fixedDeltaTime);
			CheckAndRespondCollision(rbody[i], vec3(0, 1, 0), GROUND_Y, coefficientOfRestitution, fixedDeltaTime);
			rbody[i].ApplyFriction(impulse, contactPoint, frictionCoefficient, fixedDeltaTime);
		}

		accumulator -= fixedDeltaTime; // Decrease accumulated time
	}
	// Check if the object has come to rest
	bool allResting = true;
	for (int i = 0; i < 1; ++i) {
		if (glm::length(rbody[i].Velocity()) > 0.01f || glm::length(rbody[i].AngularVelocity()) > 0.01f) { // Adjust the threshold as needed
			allResting = false;
			break;
		}
	}

	if (allResting) {
		// Set both linear and angular velocities to zero to stop the object from moving
		for (int i = 0; i < 1; ++i) {
			rbody[i].SetVelocity(vec3(0.0f));
			rbody[i].SetAngularVelocity(vec3(0.0f));
			rbody[i].ClearForcesImpulses();
		}
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


