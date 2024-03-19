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


void RigidBody::ApplyFriction(const glm::vec3& impulse, const glm::vec3& contactPoint, float frictionCoefficient, float deltaTime) {
    // First, handle the friction impulse for immediate effect
    glm::vec3 tangentDirection = glm::cross(contactPoint - Position(), impulse);
    if (glm::length(tangentDirection) > 0) {
        tangentDirection = glm::normalize(tangentDirection);
    }
    float frictionImpulseMagnitude = glm::length(impulse) * frictionCoefficient;
    glm::vec3 frictionImpulse = frictionImpulseMagnitude * tangentDirection;
    ApplyImpulse(frictionImpulse, contactPoint);

    // Then, calculate and apply the friction force for continuous effect over time
    glm::vec3 frictionForce = -glm::normalize(Velocity()) * frictionCoefficient; // Calculate the friction force as opposite to the velocity direction
    glm::vec3 deltaVelocity = frictionForce / Mass() * deltaTime; // Apply the friction force to the velocity
    glm::vec3 newVelocity = Velocity() + deltaVelocity;

    // Check if the velocity should be set to zero (stop condition)
    if (glm::length(newVelocity) < 0.01f) { // Threshold value can be adjusted
        newVelocity = glm::vec3(0.0f); // Set velocity to zero to stop the object
    }

    // Update the object's velocity
    SetVelocity(newVelocity);
}



