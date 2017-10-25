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
#include <cassert>
using namespace std;

/* Local includes */
#include "Vec2.h"
#include "Scene.h"

Point& get_other_spring_end(const Point& point, const Spring& spring)
{
    return spring.getPoint(0) == &point
        ? *spring.getPoint(1)
        : *spring.getPoint(0);
}

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

Vec2 compute_acceleration(Point& point, const vector<Spring>& springs)
{
	update_forces(point, springs);
	return compute_acceleration(point);
}

void apply_external_forces(Point& point, const bool interaction)
{
    static default_random_engine rng;
    static uniform_real_distribution<> rnd(-50, 50);

    // gravity
    static constexpr auto g = -10;

    point.setUserForce(Vec2(0, point.getMass() * g));

    if (interaction)
    {
        point.setUserForce(point.getUserForce() + Vec2(
            rnd(rng), abs(rnd(rng))));
    }
}

void euler(const double dt,
           vector<Point>& points,
           vector<Spring>& springs,
           const bool interaction)
{
	for (auto& point : points)
	{
		if (point.isFixed())
			continue;

		apply_external_forces(point, interaction);

		// x(t + h) = x(t) + h * v(t)
		// v(t + h) = v(t) + h * a(t)

		const auto new_position = point.getPos() + point.getVel() * dt;
		const auto a = compute_acceleration(point, springs);

		point.setPos(new_position);
		point.setVel(point.getVel() + a * dt);
	}
}

void symplectic(const double dt,
				vector<Point>& points,
				vector<Spring>& springs,
				const bool interaction)
{
	for (auto& point : points)
	{
		if (point.isFixed())
			continue;

        apply_external_forces(point, interaction);

		// x(t + h) = x(t) + h * v(t)
		// v(t + h) = v(t) + h * a(t + h)

		point.setPos(point.getPos() + point.getVel() * dt);

		const auto a = compute_acceleration(point, springs);

		point.setVel(point.getVel() + a * dt);
	}
}

void midpoint(const double dt, 
			  vector<Point>& points, 
			  vector<Spring>& springs, 
	          const bool interaction)
{
	for (auto& point : points)
	{
		if (point.isFixed())
			continue;

        apply_external_forces(point, interaction);

		const auto a = compute_acceleration(point, springs);

		const auto original_velocity = point.getVel();

		point.setVel(point.getVel() + dt / 2.0 * a);

		const auto original_position = point.getPos();

		point.setPos(point.getPos() + dt / 2.0 * point.getVel());

		const auto a_new = compute_acceleration(point, springs);

		point.setPos(original_position + dt * point.getVel());

		point.setVel(original_velocity + dt * a_new);
	}
}

void leapfrog(const double dt, 
			  vector<Point>& points, 
			  vector<Spring>& springs, 
	          const bool interaction)
{
	for (auto& point : points)
	{
		if (point.isFixed())
			continue;

        apply_external_forces(point, interaction);

		const auto a = compute_acceleration(point, springs);

        const auto old_velocity = point.getVel() - ( dt / 2.0 * a );
                
        const auto new_velocity = old_velocity + ( dt * a );
        
        point.setPos(point.getPos() + dt  * new_velocity);
        
        point.setVel(new_velocity);
    }
}

void analytical(const double dt,
				vector<Point>& points,
				vector<Spring>& springs,
				const bool interaction)
{
    static constexpr auto g = -10.0;

	static auto t = 0.0;
	
	t += dt;

	for(auto& point : points)
	{
        if(point.isFixed()) 
            continue;

        const auto m = point.getMass();
        const auto d = point.getDamping();

	    const auto wr = d / (2 * m);

	    for(const auto& spring : springs)
	    {
            if (spring.getPoint(0) != &point &&
                spring.getPoint(1) != &point)
            {
                continue;
            }

            const auto& spring_point = get_other_spring_end(point, spring);

            const auto connection =
                spring_point.getPos() - point.getPos();

            const auto direction = connection.normalize();

            const auto actual_gravity = g * abs(direction.y);

            const auto l = spring.getRestLength();
            const auto k = spring.getStiffness();

            const auto x0 = -l;
 
            const auto w = sqrt(k / m);

            // ensure under-damping
            assert(wr * wr  < w * w &&
                point.getDamping() < 2 * sqrt(m * spring.getStiffness()));

            const auto wbar = sqrt(w * w - wr * wr);

            const auto a = m * actual_gravity / k;

            //assert(w * cos(t * w) - wr * sin(w * t) > 0.00001 && "invalid value");

            const auto b = a * (wr / w);
                     
            // update position --------------------------------------------------------------
            const auto x = exp(-wr * t) * (a*cos(wbar * t) + b *sin(wbar * t)) - 
                m * actual_gravity / k + x0;

            point.setPos(spring_point.getPos() + x * direction);
	    }
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
			return symplectic(dt, points, springs, interaction);
		}

		case Scene::LEAPFROG:
		{
            return analytical(dt, points, springs, interaction);
            //return leapfrog(dt, points, springs, interaction);
		}

		case Scene::MIDPOINT:
		{
			return midpoint(dt, points, springs, interaction);
		}
	}
}
