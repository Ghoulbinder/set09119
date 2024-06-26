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
    float mass = Mass();


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


void RigidBody::ApplyForce(const glm::vec3& force) {
	// Directly modify the velocity based on the force applied
	glm::vec3 acceleration = force / Mass();
	SetVelocity(Velocity() + acceleration);
}








