/******************************************************************
*
* Exercise.cpp  
*
* Description: In this file - in the function TimeStep() - the various 
* numerical solvers of the first programming assignment have to be
* implemented. Feel free to add new local functions into this file. 
* Changes to other source files of the framework should not be 
* required. It is acceptable to assume knowledge about the scene
* topology (i.e. how springs and points are connected).
*
* Physically-Based Simulation Proseminar WS 2015
* 
* Interactive Graphics and Simulation Group
* Institute of Computer Science
* University of Innsbruck
*
*******************************************************************/

/* Standard includes */
#include <vector>
#include <random>
using namespace std;

/* Local includes */
#include "Vec2.h"
#include "Scene.h"

Vec2 compute_internal_forces(const Point& point, const vector<Spring>& springs)
{
	auto force = Vec2(0.0, 0.0);

	for (auto& spring : springs)
	{
		if (spring.getPoint(0) != &point &&
			spring.getPoint(1) != &point)
		{
			continue;
		}

		const auto spring_point = spring.getPoint(0) == &point
			? spring.getPoint(1)
			: spring.getPoint(0);

		const auto connection =
			point.getPos() - spring_point->getPos();

		const auto distance = connection.length();

		if (abs(distance) < 0.00000001)
			continue;

		const auto direction = connection.normalize();

		force += spring.getStiffness() *
			(spring.getRestLength() - distance) * direction;
	}

	return force;
}

Vec2 compute_acceleration(const Point& point)
{
	return (point.getForce() - point.getDamping() * point.getVel()) /
		point.getMass();
}

void update_forces(Point& point, const vector<Spring>& springs)
{
	// internal forces
	point.setForce(compute_internal_forces(point, springs));

	// external forces
	point.addForce(point.getUserForce());
}

void euler(const double dt,
           vector<Point>& points,
           vector<Spring>& springs,
           const bool interaction)
{
	static default_random_engine rng;
	static auto rnd = uniform_real_distribution<>(-50, 50);

	for (auto& point : points)
	{
		if (point.isFixed())
			continue;

		// gravity
		point.setUserForce(Vec2(0, -10));

		if (interaction)
		{
			point.setUserForce(point.getUserForce() + Vec2(
				rnd(rng), abs(rnd(rng))));
		}

		point.setPos(point.getPos() + point.getVel() * dt);

		update_forces(point, springs);

		const auto a = compute_acceleration(point);

		point.setVel(point.getVel() + a * dt);
	}
}

void midpoint(const double dt, vector<Point>& points, vector<Spring>& springs, const bool interaction)
{
	static default_random_engine rng;
	static auto rnd = uniform_real_distribution<>(-50, 50);

	for (auto& point : points)
	{
		if (point.isFixed())
			continue;

		// gravity
		point.setUserForce(Vec2(0, -10));

		if (interaction)
		{
			point.setUserForce(point.getUserForce() + Vec2(
				rnd(rng), abs(rnd(rng))));
		}

		update_forces(point, springs);

		const auto a = compute_acceleration(point);

		const auto original_velocity = point.getVel();

		point.setVel(point.getVel() + dt / 2.0 * a);

		const auto original_position = point.getPos();

		point.setPos(point.getPos() + dt / 2.0 * point.getVel());

		update_forces(point, springs);

		const auto a_new = compute_acceleration(point);

		point.setPos(original_position + dt * point.getVel());

		point.setVel(original_velocity + dt * a_new);
	}
}


/******************************************************************
*
* TimeStep
*
* This function is called every time step of the dynamic simulation.
* Positions, velocities, etc. should be updated to simulate motion
* of the mass points of the 2D scene. The selected solver is passed
* to the function as well as the time step, the springs, and the
* mass points.
*
*******************************************************************/

void TimeStep(const double dt, const Scene::Method method,
               vector<Point>& points, vector<Spring>& springs, const bool interaction)
{
	switch (method)
	{
		case Scene::EULER:
		{
			return euler(dt, points, springs, interaction);
		}

		case Scene::SYMPLECTIC:
		{
			break;
		}

		case Scene::LEAPFROG:
		{
			break;
		}

		case Scene::MIDPOINT:
		{
			return midpoint(dt, points, springs, interaction);
		}
	}
}
