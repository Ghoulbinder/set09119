#include "PhysicsEngine.h"

#include <map>
#include <numeric>

#include "Application.h"
#include "Camera.h"
#include "Force.h"

#include <glm/gtx/matrix_cross_product.hpp>
#include <glm/gtx/orthonormalize.hpp>


using namespace glm;
using namespace std;

const vec3 GRAVITY = vec3(0, -9.81, 0);

// rigid_RADIUS 
//const float RBODY_RADIUS = 0.5f; // Adjustable as needed

const float GROUND_Y = 0.0f; // since the ground is at y = 0.0f

RigidBody rbody[1];

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

//void Integrate(RigidBody& rb, float dt)
//{
//	// Apply gravity using the SetMass() method to access the mass. 
//	rb.ApplyForce(GRAVITY * rb.Mass());
//	auto pos = rb.Position() + rb.Velocity() * dt; // Use getters to access position and velocity 
//	rb.SetPosition(pos);
//}

vec3 CalculateContactPoint(const RigidBody& rb, const vec3& planeNormal, float planeDistance)
{
	// Calculate the distance from the center of the rigid body to the ground plane
	float distanceToPlane = glm::dot(rb.Position(), planeNormal) - planeDistance;

	// Calculate the contact point by projecting the center of the rigid body onto the ground plane
	vec3 contactPoint = rb.Position() - distanceToPlane * planeNormal;
	return contactPoint;
}


vec3 GroundCollisionImpulse(RigidBody& rb, float groundY, float coefficientOfRestitution) {
	vec3 particlePos = rb.Position();
	vec3 velocity = rb.Velocity();
	vec3 impulse = vec3(0.0f);

	// Check if the particle is below the ground level
	if (particlePos.y < groundY) {
		// Reflect the y component of the velocity and calculate the change
		float deltaVY = -velocity.y * coefficientOfRestitution - velocity.y;
		impulse.y = rb.Mass() * deltaVY;

		// Apply impulse considering the rotation if the collision point is off-center
		vec3 contactPoint = vec3(particlePos.x, groundY, particlePos.z); // Assuming ground contact point
		rb.ApplyImpulseWithRotation(impulse, contactPoint);

		// Position correction: move the rigid body above ground considering its radius
		// This is to ensure the object is placed just at the ground surface after collision response
		//particlePos.y = groundY + RBODY_RADIUS;
		rb.SetPosition(particlePos);
	}

	return impulse;
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
	rbody[0].SetAngularVelocity(glm::vec3(0.0f, 0.0f, 0.0f)); // Example angular velocity 

	//rbody[1].SetMesh(mesh);
	//rbody[1].SetShader(defaultShader);
	//rbody[1].SetColor(vec4(0, 1, 0, 1));
	//rbody[1].SetPosition(vec3(2.0f, 5.0f, 0.0f));
	//rbody[1].SetScale(vec3(0.5f));
	//rbody[1].SetVelocity(vec3(0.0f, 0.0f, 0.0f));
	//rbody[1].SetAngularAcceleration(glm::vec3(0.0f)); // Start with no angular acceleration 




	// TODO: Get the mesh and shader for rigidy body
	camera = Camera(vec3(0, 5, 10));
}

void PhysicsEngine::Task1Init()
{
	// Initialize the first particle as stationary
	rbody[0].SetMass(1.0f);
	rbody[0].SetPosition(vec3(0.0f, 5.0f, 0.0f));
	rbody[0].SetVelocity(vec3(0.0f, 0.0f, 0.0f));

	// Initialize the second particle to be affected by gravity
	rbody[1].SetMass(1.0f); // Ensure realistic mass
	rbody[1].SetPosition(vec3(2.0f, 5.0f, 0.0f)); // Initial position 
	rbody[1].SetVelocity(vec3(0.0f, 0.0f, 0.0f)); // Initial velocity 


}

void PhysicsEngine::Task1Update(float deltaTime, float totalTime)
{
	//Ensure forces and impulses are cleared at the start 
	for (int i = 0; i < 1; ++i) {
		rbody[i].ClearForcesImpulses();
		Force::Gravity(rbody[i]);
	}

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


	accumulator += deltaTime;

	vec3 groundNormal = vec3(0.0f, 1.0f, 0.0f); // Assuming the ground is horizontal 
	float groundDistance = 0.0f; // Assuming the ground is at y = 0 
	vector<vec3> collisionPoints;
	vector<vec3> contactPoints; // Storage for contact points

	for (int i = 0; i < 1; ++i) {
		rbody[i].ClearForcesImpulses();   // Clear forces and impulses
		// Apply forces specific to your simulation, such as gravity and spring forces
	}



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

	while (accumulator >= fixedDeltaTime) {


		for (int i = 0; i < 1; ++i) {
			// Clear forces and impulses
			rbody[i].ClearForcesImpulses();

			//// Apply gravity force
			vec3 force = GRAVITY * rbody[i].Mass();
			rbody[i].ApplyForce(force);


			// Symplectic Euler Integration
			vec3 p = rbody[i].Position();
			vec3 v = rbody[i].Velocity();
			vec3 acceleration = rbody[i].AccumulatedForce() / rbody[i].Mass();
			SymplecticEuler(p, v, rbody[i].Mass(), acceleration, vec3(0.0f), fixedDeltaTime);

			// Update position and velocity
			rbody[i].SetPosition(p);
			rbody[i].SetVelocity(v);

			// Update rotation based on angular velocity
			if (glm::length(rbody[i].AngularVelocity()) > 0.0f) {
				float angle = rbody[i].AngularVelocity().y * deltaTime; // Angular velocity.y * deltaTime for rotation angle
				glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), angle, glm::vec3(0, 1, 0)); // Create rotation matrix
				rbody[i].SetOrientation(rotation * rbody[i].Orientation()); // Apply rotation to current orientation
			}


			// Adjust position based on collision detection with ground
			AdjustRigidBodyPosition(rbody[i], vec3(0, 1, 0), GROUND_Y, coefficientOfRestitution);


		}
		accumulator -= fixedDeltaTime;
	}

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
}

// This is called every frame, after Update
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	for (int i = 0; i < 1; i++) {
		rbody[i].Draw(viewMatrix, projMatrix);
	}

	ground.Draw(viewMatrix, projMatrix);

}

void PhysicsEngine::HandleInputKey(int keyCode, bool pressed)
{
	static float timeElapsed = 0.0f;

	if (timeElapsed < 2.0f) { // Delay key effects by 2 seconds
		timeElapsed += 0.016f; // Assuming 60Hz update rate
		return;
	}

	timeElapsed = 0.0f; // Reset time elapsed for next key press

	switch (keyCode)
	{
	case 'K':  // Toggle angular acceleration for the second body
		if (pressed) {
			if (glm::length(rbody[0].AngularAcceleration()) == 0.0f) {
				rbody[0].SetAngularAcceleration(vec3(3.0f, 0.0f, 0.0f));  // Enable acceleration
			}
			else {
				rbody[0].SetAngularAcceleration(vec3(0.0f));  // Disable acceleration
				rbody[0].SetAngularVelocity(vec3(0.0f));  // Stop the moment
			}
		}
		break;
	case '1':  // Apply an impulse that will make rb move to the left with a velocity of (-1,0,0)
		if (pressed) {
			rbody[0].SetVelocity(vec3(-1.0f, 0.0f, 0.0f));
			// Optionally, reset angular velocity if needed
			// rbody[0].SetAngularVelocity(vec3(0.0f));
		}
		break;
	case '2':  // Apply impulse to move left and start spinning anticlockwise around the z axis
		if (pressed) {
			rbody[0].SetVelocity(vec3(-1.0f, 0.0f, 0.0f));
			rbody[0].SetAngularVelocity(vec3(0.0f, 0.0f, 1.0f)); // Positive Z value for anticlockwise rotation
		}
		break;
	case '3':  // Make rb stop moving
		if (pressed) {
			rbody[0].SetVelocity(vec3(0.0f, 0.0f, 0.0f));
			// Optionally, reset angular velocity if needed
			// rbody[0].SetAngularVelocity(vec3(0.0f));
		}
		break;
	case '4':  // Change direction and travel with a velocity of (-3,0,0)
		if (pressed) {
			rbody[0].SetVelocity(vec3(-3.0f, 0.0f, 0.0f));
		}
		break;
	case '5':  // Stop translating and start spinning clockwise
		if (pressed) {
			rbody[0].SetVelocity(vec3(0.0f, 0.0f, 0.0f));
			rbody[0].SetAngularVelocity(vec3(0.0f, 0.0f, -1.0f)); // Negative Z value for clockwise rotation
		}
		break;
	case '6':  // Apply an impulse that will make rb stop moving and rotating
		if (pressed) {
			rbody[0].SetVelocity(vec3(0.0f, 0.0f, 0.0f));
			rbody[0].SetAngularVelocity(vec3(0.0f, 0.0f, 0.0f));
		}
		break;
		// Add more cases as needed for additional key bindings
	default:
		break;
	
	}
}

