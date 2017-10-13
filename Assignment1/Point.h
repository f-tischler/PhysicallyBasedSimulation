/******************************************************************
*
* Point.h
*
* Description: Class definition for mass points  
*
* Physically-Based Simulation Proseminar WS 2015
* 
* Interactive Graphics and Simulation Group
* Institute of Computer Science
* University of Innsbruck
*
*******************************************************************/

#ifndef __POINT_H__
#define __POINT_H__

#include "Vec2.h"

class Point
{
private:
	Vec2 pos; /* Position of mass point */
	Vec2 vel; /* Velicity of mass point */
	Vec2 force; /* Sum of all forces on mass point */
	Vec2 userForce; /* Additional external force exerted by user */

	double mass;
	double damping;
	bool fixed; /* True, if point is fixed in space */

public: /* Various constructors */
	Point(void)
	{
		pos = Vec2(0.0, 0.0);
		vel = Vec2(0.0, 0.0);
		force = Vec2(0.0, 0.0);
		userForce = Vec2(0.0, 0.0);

		mass = 0.0;
		damping = 0.0;
		fixed = false;
	}

	Point(const Point& rhs)
	{
		pos = rhs.pos;
		vel = rhs.vel;
		force = rhs.force;
		userForce = rhs.userForce;

		mass = rhs.mass;
		damping = rhs.damping;
		fixed = rhs.fixed;
	}

	Point(Vec2 p)
	{
		pos = p;
		vel = Vec2(0.0, 0.0);
		force = Vec2(0.0, 0.0);
		userForce = Vec2(0.0, 0.0);

		mass = 0.0;
		damping = 0.0;
		fixed = false;
	}

	Point(Vec2 p, double m, double d)
	{
		pos = p;
		vel = Vec2(0.0, 0.0);
		force = Vec2(0.0, 0.0);
		userForce = Vec2(0.0, 0.0);

		mass = m;
		damping = d;
		fixed = false;
	}

	Point(double m, double d)
	{
		pos = Vec2(0.0, 0.0);
		vel = Vec2(0.0, 0.0);
		force = Vec2(0.0, 0.0);
		userForce = Vec2(0.0, 0.0);

		mass = m;
		damping = d;
		fixed = false;
	}

	~Point(void)
	{
	}

	void render();

	/* Getting and setting private variables */
	void setPos(Vec2 p);
	Vec2 getPos() const;
	void setX(double x);
	double getX() const;
	void setY(double y);
	double getY() const;

	void setVel(Vec2 v);
	Vec2 getVel() const;
	void setVelX(double vx);
	double getVelX() const;
	void setVelY(double vy);
	double getVelY() const;

	void setForce(Vec2 f);
	Vec2 getForce() const;
	void setForceX(double fx);
	double getForceX() const;
	void setForceY(double fy);
	double getForceY() const;

	void addForce(Vec2 f);

	void setMass(double m);
	double getMass() const;

	void setDamping(double d);
	double getDamping() const;

	void setFixed(bool fix);
	bool isFixed() const;

	void setUserForce(Vec2 f);
	Vec2 getUserForce() const;

	/* Copy data to new point */
	Point copy();
};

#endif
