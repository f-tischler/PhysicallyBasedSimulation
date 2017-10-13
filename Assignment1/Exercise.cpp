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
#include <iostream>
#include <vector>
using namespace std;

/* Local includes */
#include "Vec2.h"
#include "Point.h"
#include "Scene.h"



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

void euler(const double dt, 
		   vector<Point> &points,
		   vector<Spring> &springs,
           const bool interaction)
{
	for(auto& point : points)
	{
		if(point.isFixed())
			continue;

		// gravity
		point.setUserForce(Vec2(0, -10));

		if(interaction)
		{
			point.setUserForce(point.getUserForce() + Vec2(
				rand() % 50 * (rand() % 2 ? -1 : 1),
				rand() % 50));
		}

		point.setPos(point.getPos() + point.getVel() * dt);

		point.setForce(Vec2(0, 0));

		for(auto& spring : springs)
		{
			if(spring.getPoint(0) != &point &&
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
				continue;;

			const auto direction = connection.normalize();

			const auto f = spring.getStiffness() *
				(spring.getRestLength() - distance) * direction;

			point.addForce(f);
		}
		
		point.addForce(point.getUserForce());
		
		const auto a = (point.getForce() - point.getDamping() * point.getVel()) /
			point.getMass();

		point.setVel(point.getVel() + a * dt);
	}

}

void TimeStep(const double dt, const Scene::Method method, 
              vector<Point> &points, vector<Spring> &springs, const bool interaction)
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
            break;  
        }            
    }
}       

