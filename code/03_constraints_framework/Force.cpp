#include "PhysicsEngine.h"
#include "Force.h"
#include "PhysicsObject.h"

using namespace glm;

void Force::Gravity(Particle& p)
{
	auto force = vec3(0, -9.81, 0) * p.Mass();
	p.ApplyForce(force);
}

void Force::Drag(Particle& p)
{
    float dragCoefficient = 0.1f;
    float fluidDensity = 0.1f;
    glm::vec3 velocity = p.Velocity();
    float cubeSide = 1.0f; // Side length of the cube
    float area = cubeSide * cubeSide;
    glm::vec3 dragForce = -0.5f * dragCoefficient * fluidDensity * glm::length(velocity) * velocity * area;
    p.ApplyForce(dragForce);
}

void Force::Hooke(Particle& p1, Particle& p2, float restLength, float ks, float kd)
{
    glm::vec3 displacement = p2.Position() - p1.Position();
    float displacementLength = glm::length(displacement);
    glm::vec3 forceDirection = glm::normalize(displacement);

    // Spring force (ks is spring constant, kd is damping factor)
    glm::vec3 springForce = -ks * (displacementLength - restLength) * forceDirection;

    // Damping force based on relative velocity
    glm::vec3 relativeVelocity = p2.Velocity() - p1.Velocity();
    glm::vec3 dampingForce = -kd * relativeVelocity;

    // Total force applied to p2
    glm::vec3 totalForce = springForce + dampingForce;

    // Apply the force to both particles
    p2.ApplyForce(totalForce);
    p1.ApplyForce(-totalForce); // Apply equal and opposite force to p1 
}