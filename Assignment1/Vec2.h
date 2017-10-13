/******************************************************************
*
* Vec2.h
*
* Description: Code providing helper function for handling 2D
* vectors; standard operators and operators are provided  
*
* Physically-Based Simulation Proseminar WS 2015
* 
* Interactive Graphics and Simulation Group
* Institute of Computer Science
* University of Innsbruck
*
*******************************************************************/

#ifndef __VEC2_H__
#define __VEC2_H__

#include <math.h>

class Vec2
{
public:
    double x, y;

public:
    Vec2(void){x=0.0; y=0.0;}
    Vec2(double x_, double y_){x=x_; y=y_;}
    ~Vec2(void){}

    void operator+=(const Vec2& v)
    {
        x+=v.x; y+=v.y;
    }

    void operator-=(const Vec2& v)
    {
        x-=v.x; y-=v.y;
    }

    Vec2 operator+(const Vec2& v) const
    {
        return Vec2(x+v.x, y+v.y);
    }

    Vec2 operator-(const Vec2& v) const
    {
        return Vec2(x-v.x, y-v.y);
    }
   
    Vec2 operator-() const
    {
        return Vec2(-x, -y);
    }

    Vec2 operator*(const double k) const
    {
        return Vec2(k*x, k*y);
    }

    Vec2 operator/(const double k) const
    {
        return Vec2(x/k, y/k);
    }
	
    friend Vec2 operator*(double k, const Vec2& v)
    {
        return Vec2(k*v.x, k*v.y);
    }

    double dot(const Vec2& v) const
    {
        return x*v.x+y*v.y;
    }

    double cross(const Vec2& v) const
    {
        return x*v.y-y*v.x;
    }

    double length(void) const
    {
        return sqrt(x*x+y*y);
    }

    Vec2 normalize() const
    {
        return Vec2(x,y) / Vec2(x,y).length();
    }
};

#endif 
