/******************************************************************
*
* Spring.cpp
*
* Description: Implementation of functions for handling springs
*
* Physically-Based Simulation Proseminar WS 2015
* 
* Interactive Graphics and Simulation Group
* Institute of Computer Science
* University of Innsbruck
*
*******************************************************************/


#include "Spring.h"
#include <GL/freeglut.h> 


void Spring::init(Point *_p0, Point *_p1) 
{
    /* Initialize spring with pointers to both mass points */
    p0=_p0; 
    p1=_p1; 

    /* Assume rest length is given by initial configuration */
    restLength = (p0->getPos() - p1->getPos()).length();
}

void Spring::render()
{
    /* Render spring as gray line */
    glColor3f(0.5, 0.5, 0.5);
    glLineWidth(5);
    glBegin(GL_LINES);
        glVertex3d(p0->getX(), p0->getY(), 0.0);
        glVertex3d(p1->getX(), p1->getY(), 0.0);
    glEnd();
}

void Spring::setRestLength(double L)
{
    restLength = L;
}

double Spring::getRestLength()
{
    return restLength;
}

void Spring::setStiffness(double k)
{
    stiffness = k;
}

double Spring::getStiffness()
{
    return stiffness;
}

Point *Spring::getPoint(int i)
{
    /* Return point 1 or 0 (assume i = 0,1) */
    if(i)
        return p1;
    else
        return p0;
}
