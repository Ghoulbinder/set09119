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

glm::mat3 RigidBody::InverseInertia() {
    float mass = Mass(); // Assuming Mass() is a method that returns the mass of the sphere
    float radius = Scale().x; // Assuming uniform scaling and that Scale().x gives the radius

    // For a solid sphere, the moment of inertia is (2/5) * mass * radius^2
    // Note: This assumes a uniform scaling factor for x, y, and z (i.e., a perfect sphere)
    float inertia = (2.0f / 5.0f) * mass * radius * radius;

    glm::mat3 inertiaTensor(0.0f);
    inertiaTensor[0][0] = inertia; // Ixx
    inertiaTensor[1][1] = inertia; // Iyy
    inertiaTensor[2][2] = inertia; // Izz

    // The orientation does not affect the inertia tensor for a sphere as it's isotropic (the same in all directions)
    // Therefore, we can directly return the inverse of the inertia tensor without adjusting for orientation
    return glm::inverse(inertiaTensor);
}

