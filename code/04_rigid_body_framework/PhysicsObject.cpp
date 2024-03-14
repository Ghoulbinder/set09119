#include "PhysicsObject.h"

#include <glm/glm.hpp>

#include "PhysicsEngine.h"
#include "Mesh.h"
#include "Shader.h"


void PhysicsBody::Draw(const glm::mat4& viewMatrix, const glm::mat4& projectionMatrix) const
{
	m_shader->Use();
	m_shader->SetUniform("model", ModelMatrix());
	m_shader->SetUniform("view", viewMatrix); 
    m_shader->SetUniform("projection", projectionMatrix);
	m_shader->SetUniform("color", m_color);  

	auto mvp = projectionMatrix * viewMatrix * ModelMatrix();
	m_shader->SetUniform("modelViewProjectionMatrix", mvp);
	m_shader->SetUniform("normalMatrix", transpose(inverse(viewMatrix * ModelMatrix())));
	m_mesh->DrawVertexArray();
}

glm::mat3 RigidBody::InverseInertia()
{
    float mass = 1.0f; // Mass m = 1


    // Since the cuboid is scaled by (1, 3, 1), the dimensions are effectively (width = 2, height = 6, depth = 2)
    float width = 2.0f * Scale().x;  // x-axis scaling
    float height = 6.0f * Scale().y; // y-axis scaling, adjusted to match your specification
    float depth = 2.0f * Scale().z;  // z-axis scaling

    // Calculate the components of the inertia tensor for a cuboid
   // Computing the inertia tensor based on the cuboid formula
    glm::mat3 inertiaTensor(0.0f);
    inertiaTensor[0][0] = (1.0f / 12.0f) * mass * (height * height + depth * depth); // Ixx
    inertiaTensor[1][1] = (1.0f / 12.0f) * mass * (width * width + depth * depth);   // Iyy
    inertiaTensor[2][2] = (1.0f / 12.0f) * mass * (width * width + height * height); // Izz

    // Extract the rotational part of the model matrix (orientation)
    glm::mat3 orientation = glm::mat3(Orientation()); // Assuming Orientation() returns a mat4 and is the current orientation

    // Rotate the inertia tensor to world space
    glm::mat3 worldInertiaTensor = orientation * inertiaTensor * glm::transpose(orientation);

    // Calculate and return the inverse of the world space inertia tensor
    return glm::inverse(worldInertiaTensor);

}

void RigidBody::ApplyImpulse(const glm::vec3& impulse, const glm::vec3& contactPoint)
{
	// Calculate linear velocity change
	glm::vec3 deltaVelocity = impulse / Mass();

	// Update linear velocity
	SetVelocity(Velocity() + deltaVelocity);

	// Calculate angular velocity change using torque
	glm::vec3 r = contactPoint - Position();
	glm::vec3 deltaAngularVelocity = glm::inverse(InverseInertiaTensor()) * glm::cross(r, impulse);

	// Update angular velocity
	SetAngularVelocity(AngularVelocity() + deltaAngularVelocity);
}
void RigidBody::ApplyAngularDrag(float deltaTime) {
	// Assuming you have a way to get the angular drag coefficient. If not, define it.
	float angularDragCoefficient = 0.05f; // This value is an example.

	// Calculate the magnitude of the angular velocity.
	float angularVelocityMagnitude = glm::length(m_angularVelocity);

	// Calculate the new magnitude after applying drag.
	// This formula can vary depending on how you want to simulate drag.
	float newMagnitude = angularVelocityMagnitude - angularDragCoefficient * angularVelocityMagnitude * deltaTime;

	// Ensure the new magnitude is not negative.
	newMagnitude = glm::max(newMagnitude, 0.0f);

	// Calculate the new angular velocity with the original direction but new magnitude.
	if (angularVelocityMagnitude > 0.0f) { // Avoid division by zero.
		glm::vec3 newAngularVelocity = (m_angularVelocity / angularVelocityMagnitude) * newMagnitude;
		m_angularVelocity = newAngularVelocity;
	}
}


 

void RigidBody::ApplyDrag(float deltaTime) {
	// Linear drag
	float dragCoefficient = 0.47f; // This is an example value
	glm::vec3 linearDragForce = -dragCoefficient * GetVelocity();
	ApplyForce(linearDragForce);

	// Angular drag
	float angularDragCoefficient = 0.05f; // Example value
	glm::vec3 angularDragForce = -angularDragCoefficient * m_angularVelocity;
	// Directly modify angular velocity for simplicity
	m_angularVelocity += angularDragForce * deltaTime;
}


void RigidBody::ApplyForce(const glm::vec3& force) {
	// Directly modify the velocity based on the force applied
	glm::vec3 acceleration = force / Mass();
	SetVelocity(Velocity() + acceleration);
}

// This function calculates the impulse, including rotational effects, and applies it to the rigid body.
void RigidBody::ApplyImpulseWithRotation(const glm::vec3& impulse, const glm::vec3& contactPoint) {
	glm::vec3 r = contactPoint - Position(); // Vector from CM to contact point
	ApplyImpulse(impulse, contactPoint); // Apply linear impulse
	glm::vec3 angularImpulse = glm::cross(r, impulse);
	glm::vec3 deltaAngularVelocity = m_inverseInertiaTensor * angularImpulse;
	m_angularVelocity += deltaAngularVelocity;
}
void RigidBody::ApplyFrictionImpulse(const glm::vec3& impulse, const glm::vec3& contactPoint, float frictionCoefficient) {
	glm::vec3 tangentDirection = glm::cross(contactPoint - Position(), impulse);
	if (glm::length(tangentDirection) > 0)
		tangentDirection = glm::normalize(tangentDirection);

	float frictionImpulseMagnitude = glm::length(impulse) * frictionCoefficient;
	glm::vec3 frictionImpulse = frictionImpulseMagnitude * tangentDirection;

	ApplyImpulse(frictionImpulse, contactPoint);
}


