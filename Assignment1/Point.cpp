/******************************************************************
*
* Point.cpp
*
* Description: Implementation of functions for handling mass points  
*
* Physically-Based Simulation Proseminar WS 2015
* 
* Interactive Graphics and Simulation Group
* Institute of Computer Science
* University of Innsbruck
*
*******************************************************************/

#include "Point.h"
#include <GL/freeglut.h>

void Point::render()
{
	/* Fixed vertices displayed in blue, free in red */
	if (fixed)
		glColor3f(0.0, 0.0, 1.0);
	else
		glColor3f(1.0, 0.0, 0.0);

	/* Assume 2D scene, Z-axis disregarded */
	glTranslatef(
		static_cast<float>(pos.x),
		static_cast<float>(pos.y),
		0.0);

	/* Draw unshaded spheres; appear as filled circles */
	glutSolidSphere(0.1, 36, 36);
	glLoadIdentity();
}

void Point::setPos(Vec2 p)
{
	pos = p;
}

Vec2 Point::getPos() const
{
	return pos;
}

void Point::setX(double x)
{
	pos.x = x;
}

double Point::getX() const
{
	return pos.x;
}

void Point::setY(double y)
{
	pos.y = y;
}

double Point::getY() const
{
	return pos.y;
}

void Point::setVel(Vec2 v)
{
	vel = v;
}

Vec2 Point::getVel() const
{
	return vel;
}

void Point::setVelX(double vx)
{
	vel.x = vx;
}

double Point::getVelX() const
{
	return vel.x;
}

void Point::setVelY(double vy)
{
	vel.y = vy;
}

double Point::getVelY() const
{
	return vel.y;
}

void Point::setForce(Vec2 f)
{
	force = f;
}

Vec2 Point::getForce() const
{
	return force;
}

void Point::setForceX(double fx)
{
	force.x = fx;
}

double Point::getForceX() const
{
	return force.x;
}

void Point::setForceY(double fy)
{
	force.y = fy;
}

double Point::getForceY() const
{
	return force.y;
}

void Point::addForce(Vec2 f)
{
	force = force + f;
}

void Point::setMass(double m)
{
	mass = m;
}

double Point::getMass() const
{
	return mass;
}

void Point::setDamping(double d)
{
	damping = d;
}

double Point::getDamping() const
{
	return damping;
}

void Point::setFixed(bool fix)
{
	fixed = fix;
}

bool Point::isFixed() const
{
	return fixed;
}

void Point::setUserForce(Vec2 f)
{
	userForce = f;
}

Vec2 Point::getUserForce() const
{
	return userForce;
}
