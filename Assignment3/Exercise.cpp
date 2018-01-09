/******************************************************************
*
* Exercise.cpp
*
* Description: In this file the functions AdvectWithSemiLagrange(),
* SolvePoisson(), and CorrectVelocities() have to be implemented
* for the third programming assignment.
* Feel free to add new local functions into this file.
* Changes to other source files of the framework should not be
* required. 
*
* Physically-Based Simulation Proseminar WS 2016
*
* Interactive Graphics and Simulation Group
* Institute of Computer Science
* University of Innsbruck
*
*******************************************************************/


#include "Fluid2D.h"
#include "Vec2.h"
#include <iostream>
#include <math.h>
#include <algorithm>
#include <cmath>


static int xRes_static = -1;
constexpr double h=3.0f;
constexpr double h2=9.0f;


int index(int x, int y)
{
    return y * xRes_static + x;
}

void set_point(double* field, int x, int y,double value)
{
    int i=index(x, y);
    field[i] = value;
}

double mix(double x, double y, double alpha)
{
    return x * (1.0f - alpha) + (y * alpha);
}

double fastModf(double x, double &part)
{
    part=floor(x);
    return x - part;
}

double sampleTrilinear(double *field, double x, double y) {
    double xIntPart, yIntPart;

    double xFloatPart = fastModf(x,xIntPart);
    double yFloatPart = fastModf(y,yIntPart);

    xIntPart=floor(xIntPart);
    yIntPart=floor(yIntPart);

    double tmp1{0.0f}, tmp2{0.0f}, tmp3{0.0f}, tmp4{0.0f},
            tmp12{0.0f},tmp34{0.0f};

    tmp1 = field[ index( xIntPart, yIntPart ) ];
    tmp2 = field[ index( xIntPart + 1, yIntPart ) ];
    tmp3 = field[ index( xIntPart, yIntPart + 1 ) ];
    tmp4 = field[ index( xIntPart + 1, yIntPart + 1 ) ];


    tmp12 = mix(tmp1, tmp2, xFloatPart);
    tmp34 = mix(tmp3, tmp4, xFloatPart);

    return mix(tmp12, tmp34, yFloatPart);
}

void AdvectWithSemiLagrange(int xRes, int yRes, double dt,
                            double *xVelocity, double *yVelocity,
                            double *field, double* tempField)
{
    if(xRes_static==-1)
        xRes_static=xRes;
    for (int y = 1; y < yRes - 1; ++y) {
        for (int x = 1; x < xRes - 1; ++x) {

            double xOffset = (double)x - (xVelocity[index(x,y)] * dt);
            double yOffset = (double)y - (yVelocity[index(x,y)] * dt);

            xOffset = std::min((double) (xRes - 2),
                               std::max((double) 1.0f, xOffset));
            yOffset = std::min((double) (yRes - 2),
                               std::max((double) 1.0f, yOffset));

            double newValue = sampleTrilinear(field, xOffset, yOffset);

            set_point(tempField, x, y, newValue);
        }
    }

}


void SolvePoisson(int xRes, int yRes, int iterations, double accuracy,
                  double* pressure, double* divergence)
{
    double *tempPressure=(double*)malloc(sizeof(double)*xRes*yRes);
    for (int y = 0; y < yRes*xRes; ++y)
        tempPressure[y]=pressure[y];
    double error=0;

    int iteration=0;
    while (true) {
        iteration+=1;
        for (int y = 1; y < yRes - 1; ++y) {
            for (int x = 1; x < xRes - 1; ++x) {

                double newValue = ( divergence[index(x,y)] +
                                    ( pressure[index(x + 1, y)] +
                                      pressure[index(x, y + 1)] +
                                      pressure[index(x - 1, y)] +
                                      pressure[index(x, y - 1)]
                                    )
                                  ) / 4.0f;

                error += abs(pressure[index(x,y)] - newValue);
                set_point(pressure,x,y,newValue);
            }
        }


        if( error < accuracy)
            break;
        if(iteration >= iterations)
            break;
    }
}

void CorrectVelocities(int xRes, int yRes, double dt, const double* pressure,
                       double* xVelocity, double* yVelocity)
{

    for (int y = 1; y < yRes; ++y) {
        for (int x = 1; x < xRes; ++x) {
            double presx2y=pressure[index(x + 1,y)];
            double presxy2=pressure[index(x,y + 1)];
            double presx1y=pressure[index(x - 1,y)];
            double presxy1=pressure[index(x,y - 1)];
            double presx=presx2y-presx1y;
            double presy=presxy2-presxy1;

            double deltaxVel = (dt / 1.0f) * (1.0f / h) * presx;
            double deltayVel = (dt / 1.0f) * (1.0f / h) * presy;
            xVelocity[index(x,y)] = xVelocity[index(x,y)] - deltaxVel;
            yVelocity[index(x,y)] = yVelocity[index(x,y)] - deltayVel;
        }
    }
}
