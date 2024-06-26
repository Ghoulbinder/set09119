#include "PhysicsEngine.h"
#include "Force.h"
#include "PhysicsObject.h"

using namespace glm;

void Force::Gravity(Particle& rb)
{

	auto force = vec3(0, -9.81, 0) * rb.Mass();
	rb.ApplyForce(force);
}

void Force::Drag(Particle& p)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Should apply the aerodynamic drag force
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

void Force::Hooke(Particle& p1, Particle& p2, float restLength, float ks, float kd)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Should return the aerodynamic drag force
}
