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
bool scenarioApplied[9] = { false }; // Initialize all scenario flags as false. Adjust the size based on the number of scenarios.

//DO NOT SET VELOCITY AND POSISTION TO 0 WHEN CALCULATING THE IMMPULSE

void ExplicitEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{// Apply the impulse to velocity directly. Impulse is force applied over a short time.
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



vec3 CalculateImpulseWithRotation(RigidBody& rb, const vec3& pointOfCollision, const vec3& collisionNormal, float restitution) {
	// Relative velocity at point of collision
	vec3 r = pointOfCollision - rb.Position(); // Assuming CenterOfMass is essentially Position
	vec3 velocityAtPoint = rb.Velocity() + glm::cross(rb.AngularVelocity(), r);

	// Impulse calculation
	vec3 impulseDirection = -velocityAtPoint;
	float jNumerator = glm::dot(-(1 + restitution) * impulseDirection, collisionNormal);
	float jDenominator = glm::dot(collisionNormal, collisionNormal) * (1 / rb.Mass()) +
		glm::dot(glm::cross(rb.InverseInertiaTensor() * glm::cross(r, collisionNormal), r), collisionNormal);

	vec3 impulse = jNumerator / jDenominator * collisionNormal;

	// Update linear and angular velocities
	rb.ApplyImpulse(impulse, pointOfCollision);

	return impulse;
}

void AdjustRigidBodyPosition(RigidBody& rb, const vec3& groundNormal, float groundY, float coefficientOfRestitution) {
	const auto& meshData = rb.GetMesh()->Data();
	const auto& modelMatrix = rb.ModelMatrix();
	float deepestPenetration = 0.0f;

	// First pass to find the deepest penetration
	for (const auto& vertex : meshData.positions.data) {
		glm::vec4 worldVertex = modelMatrix * glm::vec4(vertex, 1.0f);
		float distanceToGround = worldVertex.y - groundY;
		if (distanceToGround < deepestPenetration) {
			deepestPenetration = distanceToGround;
		}
	}

	// Correct the position based on the deepest penetration
	if (deepestPenetration < 0) {
		vec3 correction = abs(deepestPenetration) * groundNormal;
		rb.SetPosition(rb.Position() + correction);

		// Reflect velocity if moving towards the ground
		vec3 velocity = rb.Velocity();
		if (velocity.y < 0) {
			velocity.y = -velocity.y * coefficientOfRestitution;
			rb.SetVelocity(velocity);
		}
	}

	// Second pass to adjust for any remaining penetrations
	for (const auto& vertex : meshData.positions.data) {
		glm::vec4 worldVertex = modelMatrix * glm::vec4(vertex, 1.0f);
		float distanceToGround = worldVertex.y - groundY;
		if (distanceToGround < 0) {
			vec3 additionalCorrection = abs(distanceToGround) * groundNormal;
			rb.SetPosition(rb.Position() + additionalCorrection);
			break; // Adjust once for any remaining penetration
		}
	}
}

std::pair<vec3, vec3> CalculateVelocityAndAngularRotation(const vec3& currentVelocity, const vec3& currentAngularRotation, const vec3& gravity, const vec3& angularAcceleration, float dt)
{
	
	// Calculate the acceleration due to gravity acting on the object
	vec3 acceleration = GRAVITY;

	// Calculate the new velocity using Symplectic Euler integration
	vec3 newVelocity = currentVelocity + acceleration * dt;

	// Calculate the new angular rotation using Symplectic Euler integration
	vec3 newAngularRotation = currentAngularRotation + angularAcceleration * dt;

	
	// Return the new velocity and angular rotation as a pair
	return { newVelocity, newAngularRotation };
}



// A helper function to convert a vec3 to a string
std::string vec3_to_string(const glm::vec3& vec) {
	return "(" + std::to_string(vec.x) + ", " + std::to_string(vec.y) + ", " + std::to_string(vec.z) + ")";
}

void ApplyGravityAndIntegrate(RigidBody& rb, const vec3& gravity, float deltaTime) {
	// Apply linear drag
	float linearDragCoefficient = 0.47f; // Example value, adjust based on object characteristics
	vec3 velocity = rb.GetVelocity(); 
	vec3 linearDragForce = -linearDragCoefficient * glm::length(velocity) * velocity; 
	rb.ApplyForce(linearDragForce); 

	// Apply gravity force to the rigid body
	vec3 gravitationalForce = gravity * rb.Mass();
	rb.ApplyForce(gravitationalForce);

	// Apply angular drag 
	float angularDragCoefficient = 0.05; // This is a made-up coefficient for demonstration
	// Calculate the magnitude of the angular velocity.
	float angularVelocityMagnitude = glm::length(rb.GetAngularVelocity());
	// Calculate the new magnitude after applying drag. 
	float newMagnitude = angularVelocityMagnitude - angularDragCoefficient * angularVelocityMagnitude * deltaTime;
	// Ensure the new magnitude is not negative.
	newMagnitude = glm::max(newMagnitude, 0.0f);
	// Calculate the new angular velocity with the o riginal direction but new magnitude.
	if (angularVelocityMagnitude > 0.0f) { // Avoid division by zero.
		glm::vec3 newAngularVelocity = (rb.GetAngularVelocity() / angularVelocityMagnitude) * newMagnitude;
		rb.SetAngularVelocity(newAngularVelocity);
	}


	// Integration step for position and velocity 
	vec3 accumulatedForce = rb.AccumulatedForce(); 
	vec3 acceleration = accumulatedForce / rb.Mass(); 

	vec3 currentVel = velocity + acceleration;
	// Use Symplectic Euler for integration
	vec3 newPos = rb.Position() + velocity * deltaTime; 
	vec3 newVel = currentVel + acceleration * deltaTime;  

	// Update the rigid body's position and velocity
	rb.SetPosition(newPos); 
	rb.SetVelocity(newVel); 
}





 

float adjustRestitutionBasedOnVelocity(const glm::vec3& relativeVelocity) {
	const float threshold = 2.0f; // Velocity threshold
	if (glm::length(relativeVelocity) < threshold) {
		return 0.5f; // Less restitution for slow impacts
	}
	return 0.9f; // Higher restitution for fast impacts
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

void PhysicsEngine::Task1Init()
{
	// Initialize the first particle as stationary
	rbody[0].SetMass(1.0f);
	rbody[0].SetPosition(vec3(0.0f, 5.0f, 0.0f));
	rbody[0].SetVelocity(vec3(0.0f, 0.0f, 0.0f));


}

void PhysicsEngine::Task1Update(float deltaTime, float totalTime)
{
	

	// Integration step for both particles
	for (int i = 0; i < 1; ++i) {
		// Retrieve accumulated force and impulse
		vec3 accumulatedForce = rbody[i].AccumulatedForce();
		vec3 accumulatedImpulse = rbody[i].AccumulatedImpulse();

		// Calculate acceleration
		vec3 acceleration = (accumulatedForce + accumulatedImpulse) / rbody[i].Mass();

		// Symplectic Euler Integration for velocity and position
		rbody[i].SetVelocity(rbody[i].Velocity() + acceleration * deltaTime);
		rbody[i].SetPosition(rbody[i].Position() + rbody[i].Velocity() * deltaTime);
	}
	
}


// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{
	
	

		static double accumulator = 0.0;
		const double fixedDeltaTime = 0.016; // 16ms for a 60Hz update rate 
		float coefficientOfRestitution = 0.9f;
		const double delayTime = 2.0; // Delay time in seconds 

		accumulator += deltaTime;
		totalTime += deltaTime;

		vec3 groundNormal = vec3(0.0f, 1.0f, 0.0f); //ground is horizontal 
		float groundDistance = 0.0f; //the ground is at y = 0 
		vector<vec3> collisionPoints;
		vector<vec3> contactPoints; // Storage for contact points

		

		// Handle angular integration for each rigid body
		for (int i = 0; i < 1; ++i) {
			// Handle angular integration
			auto newAngVel = rbody[i].AngularVelocity() + deltaTime * rbody[i].AngularAcceleration();
			rbody[i].SetAngularVelocity(newAngVel);

			// Create skew-symmetric matrix for angular velocity
			mat3 angVelSkew = matrixCross3(newAngVel);

			// Create 3x3 rotation matrix from rigid body orientation
			mat3 R = mat3(rbody[i].Orientation());

			// Update rotation matrix
			R += deltaTime * angVelSkew * R;
			R = glm::orthonormalize(R);

			rbody[i].SetOrientation(glm::mat4(R));
		}

		// Perform fixed time step integration using Symplectic Euler method
		while (accumulator >= fixedDeltaTime) {


			for (int i = 0; i < 1; ++i) {
				// Clear forces and impulses
				rbody[i].ClearForcesImpulses();

				

				// Update rotation based on angular velocity
				if (glm::length(rbody[i].AngularVelocity()) > 0.0f) {
					float angle = rbody[i].AngularVelocity().y * deltaTime; // Angular velocity.y * deltaTime for rotation angle
					glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), angle, glm::vec3(0, 1, 0)); // Create rotation matrix
					rbody[i].SetOrientation(rotation * rbody[i].Orientation()); // Apply rotation to current orientation
				}

				ApplyGravityAndIntegrate(rbody[i], GRAVITY, fixedDeltaTime); 
				// Adjust position based on collision detection with ground
				AdjustRigidBodyPosition(rbody[i], vec3(0, 1, 0), GROUND_Y, coefficientOfRestitution);


			}
			accumulator -= fixedDeltaTime;
		}

		// Collision detection and response with the ground
		for (auto& rb : rbody) {

			if (!collisionPoints.empty()) {
				// Calculate and apply the collision response here
				// For simplicity, we're assuming all collisions are with the ground plane
				vec3 contactPoint = CalculateContactPoint(rb, groundNormal, groundDistance);  // Implement this function based on your collision detection algorithm
				vec3 collisionNormal = vec3(0, 1, 0); // Upward ground normal
				vec3 velocityAtPoint = rb.Velocity(); // Assuming the velocity at the collision point is the body's velocity
				float restitution = 0.9f; // Coefficient of restitution (bounciness)
				vec3 impulse = CalculateImpulseWithRotation(rb, contactPoint, collisionNormal, restitution);
				rb.ApplyImpulse(impulse, contactPoint); // Pass the contact point to the function
			}
		}

		 
		//Apply an impulse that will make rb move to the left with a velocity of(-1, 0, 0)
		if (scenario >= 0 && scenario <= 8) {

			//Give rb an initial velocity of(0, 0, 0) and an angular velocity of(0, 0, 0).In
			//other words, rb is immobile!
			if (scenario == 0) {

				rbody[0].ClearForcesImpulses();
				rbody[0].SetPosition(vec3(0.0f, 5.0f, 0.0f));
				rbody[0].SetVelocity(vec3(0.0f, 0.0f, 0.0f));
				rbody[0].SetAngularVelocity(vec3(0.0f, 0.0f, 0.0f));

			}

			//– Apply an impulse that will make rb move to the left 
			// with a velocityof(-1, 0, 0)
		    else if (scenario == 1 && !scenarioApplied[1]) {
					
				if (!scenarioApplied[1]) {
					//rbody[0].ClearForcesImpulses();

					// Apply impulses
					rbody[0].SetVelocity(vec3(-1.0f, 0.0f, 0.0f));

					rbody[0].SetAngularVelocity(vec3(0.0f, 0.0f, 0.0f));

					// Calculate new velocity and angular rotation
					auto newVelAndAngRot = CalculateVelocityAndAngularRotation(rbody[0].Velocity(), rbody[0].AngularVelocity(), GRAVITY, vec3(0.0f), fixedDeltaTime);

					scenarioApplied[1] = true;
				}



				ApplyGravityAndIntegrate(rbody[0], GRAVITY, fixedDeltaTime); 

				// Adjust position based on collision detection with ground
				AdjustRigidBodyPosition(rbody[0], vec3(0, 1, 0), GROUND_Y, coefficientOfRestitution);

			}
			//Apply an impulse that will make rb move to the left with a velocity of(-1, 0, 0) 
			// and start spinning anticlockwise around the z axis

			if (scenario == 2) { 
				if (!scenarioApplied[2]) { 
					if (totalTime >= 2.0) { // Checks if 2 seconds have passed
						rbody[0].ClearForcesImpulses(); // Clears existing forces and impulses

						// Set the desired velocity and angular velocity
						rbody[0].SetVelocity(vec3(-1.0f, 0.0f, 0.0f)); // Move left
						rbody[0].SetAngularVelocity(vec3(0.0f, 0.0f, 1.0f)); // Spin anticlockwise

						scenarioApplied[2] = true; // Prevents re-application in subsequent updates
					}
				}
				
				AdjustRigidBodyPosition(rbody[0], vec3(0, 1, 0), GROUND_Y, coefficientOfRestitution);
			}
			if (scenario == 3) {
				if (!scenarioApplied[3]) {
					if (totalTime >= 2.0) { // Check if 2 seconds have passed since scenario 3 started
						rbody[0].ClearForcesImpulses(); // Clear any existing forces and impulses

						// Step 1: Make rb stop moving
						// rbody[0].SetVelocity(vec3(0.0f, 0.0f, 0.0f));

						// Step 2: Change direction and move left with (-3,0,0)
						// rbody[0].SetVelocity(vec3(-3.0f, 0.0f, 0.0f));

						// Step 3: Stop translating and start spinning clockwise
						rbody[0].SetVelocity(vec3(0.0f, 0.0f, 0.0f)); // Stop translating
						rbody[0].SetAngularVelocity(vec3(0.0f, 0.0f, -1.0f)); // Start spinning clockwise

						scenarioApplied[3] = true; // Mark scenario as applied to prevent reapplication
					}
				}
				
				AdjustRigidBodyPosition(rbody[0], vec3(0, 1, 0), GROUND_Y, coefficientOfRestitution);
			}

			if (scenario == 4) {
				if (!scenarioApplied[4]) {
					if (totalTime >= 2.0) { // Checks if 2 seconds have passed
						rbody[0].ClearForcesImpulses(); // Clears existing forces and impulses

						// Apply the changes to make the rb stop moving and rotating
						rbody[0].SetVelocity(vec3(0.0f, 0.0f, 0.0f)); // Stops moving
						rbody[0].SetAngularVelocity(vec3(0.0f, 0.0f, 0.0f)); // Stops rotating

						scenarioApplied[4] = true; // Prevents re-application in subsequent updates
					}
				}
				// Optionally adjust the rigid body's position based on collisions
				AdjustRigidBodyPosition(rbody[0], vec3(0, 1, 0), GROUND_Y, coefficientOfRestitution);
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
	if (!pressed) return; // Only handle keydown events

	// Determine if the scenario changes based on the keyCode,
	// and reset all flags to false if it does.
	int prevScenario = scenario;
	switch (keyCode) {
	case '0': scenario = 0; break;
	case '1': scenario = 1; break;
	case '2': scenario = 2; break;
	case '3': scenario = 3; break;
	case '4': scenario = 4; break;
	
		// Add more cases as needed...
	}

	// If the scenario has changed, reset all applied flags
	if (prevScenario != scenario) { 
		fill(begin(scenarioApplied), end(scenarioApplied), false); // Reset all flags 
		scenarioApplied[scenario] = false; // Ensure the new scenario is marked as not applied
	}
}


