/******************************************************************
*
* Spring.h
*
* Description: Class definition for elastic springs
*
* Physically-Based Simulation Proseminar WS 2015
* 
* Interactive Graphics and Simulation Group
* Institute of Computer Science
* University of Innsbruck
*
*******************************************************************/

#ifndef __SPRING_H__
#define __SPRING_H__

#include "Vec2.h"
#include "Point.h"

class Spring
{
private:
    Point *p0, *p1;      /* Pointer to two end points */
    double stiffness;
    double restLength;   /* Rest length of spring (does not have to be initial length) */

public:                  /* Various constructors */ 
    Spring(void)
    {
        p0 = new Point;
        p1 = new Point;
        stiffness = 0.0;
        restLength = 0.0;
    }
 
    Spring(double k)
    {
        p0 = new Point;
        p1 = new Point;
        stiffness = k;
        restLength = 0.0;
    }

    ~Spring(void){}
	
    void init(Point *_p0, Point *_p1);
    void render();

    void setRestLength(double L);
    double getRestLength();

    void setStiffness(double k);
    double getStiffness();

    Point *getPoint(int i);     /* Return end point 0 or 1 */
};

#endif

