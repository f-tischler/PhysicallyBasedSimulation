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

int index(Vector2 pos)
{
    return pos.y() * xRes_static + pos.x();
}

void set_point(double* field, Vector2 pos,double value)
{
    int i=index(pos);
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

    double tmp1{0.0f}, tmp2{0.0f}, tmp3{0.0f}, tmp4{0.0f},
            tmp12{0.0f},tmp34{0.0f};

    tmp1 = field[ index( {(int)xIntPart, (int)yIntPart} ) ];
    tmp2 = field[ index( {(int)xIntPart + 1, (int)yIntPart} ) ];
    tmp3 = field[ index( {(int)xIntPart, (int)yIntPart + 1} ) ];
    tmp4 = field[ index( {(int)xIntPart + 1, (int)yIntPart + 1} ) ];


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
            Vector2 cur_cell(x, y);
            double xComp = xVelocity[index(cur_cell)];
            double yComp = yVelocity[index(cur_cell)];

            double xOffset = x - (xComp * dt);
            double yOffset = y - (yComp * dt);

            xOffset = std::min((double) (xRes - 2),
                               std::max((double) 1.0f, xOffset));
            yOffset = std::min((double) (yRes - 2),
                               std::max((double) 1.0f, yOffset));

            double newValue = sampleTrilinear(field, xOffset, yOffset);

            set_point(tempField, cur_cell, newValue);
        }
    }

}


void SolvePoisson(int xRes, int yRes, int iterations, double accuracy,
                  double* pressure, double* divergence)
{
    double *tempPressure=(double*)malloc(sizeof(double)*xRes*yRes);

    for (int y = 0; y < yRes*xRes; ++y)
        tempPressure[y]=0.0f;

    while (true) {
        double error=0;
        for (int y = 0; y < yRes; ++y) {
            for (int x = 0; x < xRes; ++x) {
                Vector2 cur_cell({x,y});
                double current = pressure[index(cur_cell)];
                //Using five points as it is necessary to take the local value into account as well
                double newValue = ( current +
                                    pressure[index({x + 1, y})] +
                                    pressure[index({x, y + 1})] +
                                    pressure[index({x - 1, y})] +
                                    pressure[index({x, y - 1})]
                                  ) / 5.0f;

                set_point(tempPressure,cur_cell,newValue);
                error += current - newValue > 0 ? current - newValue : newValue - current;
            }
        }
        if(error < accuracy)
            break;
        for (int y = 0; y < yRes; ++y) {
            for (int x = 0; x < xRes; ++x) {
                Vector2 cur_cell({x,y});
                double current = pressure[index(cur_cell)];
                //Using five points as it is necessary to take the local value into account as well
                double newValue = ( current +
                                    tempPressure[index({x + 1, y})] +
                                    tempPressure[index({x, y + 1})] +
                                    tempPressure[index({x - 1, y})] +
                                    tempPressure[index({x, y - 1})] -
                                    divergence[index({x,y})]
                                  ) / 5.0f;

                set_point(pressure,cur_cell,newValue);
                error += current - newValue > 0 ? current - newValue : newValue - current;
            }
        }
        if(error < accuracy)
            break;
    }
}

void CorrectVelocities(int xRes, int yRes, double dt, const double* pressure,
                       double* xVelocity, double* yVelocity)
{

    double* xVelocityTemp = new double[yRes*xRes];
    double* yVelocityTemp = new double[yRes*xRes];
    for (int y = 0; y < yRes*xRes; ++y) {
        xVelocityTemp[y] = xVelocity[y] * 0.9f;
        yVelocityTemp[y] = yVelocity[y] * 0.9f;
    }

    for (int y = 0; y < yRes; ++y) {
        for (int x = 0; x < xRes; ++x) {
            Vector2 cur_cell(x, y);

            double xComp = xVelocity[index(cur_cell)];
            double yComp = yVelocity[index(cur_cell)];

            double xOffset = x + (xComp * dt);
            double yOffset = y + (yComp * dt);

            xOffset = std::min((double) (xRes - 2),
                               std::max((double) 1.0f, xOffset));
            yOffset = std::min((double) (yRes - 2),
                               std::max((double) 1.0f, yOffset));

            double xIntPart, yIntPart;

            double xFloatPart = fastModf(xOffset,xIntPart);
            double yFloatPart = fastModf(yOffset,yIntPart);

            double xInverseFloatPart = 1 - xFloatPart;
            double yInverseFloatPart = 1 - yFloatPart;

            int *indexes = new int[4];
            indexes[0] = index( {(int)xIntPart, (int)yIntPart} );
            indexes[1] = index( {(int)xIntPart, (int)yIntPart + 1} );
            indexes[2] = index( {(int)xIntPart + 1, (int)yIntPart} );
            indexes[3] = index( {(int)xIntPart + 1, (int)yIntPart + 1} );

            double *xValues = new double[4];
            double *yValues = new double[4];
            xValues[0] = ( xComp * ((xInverseFloatPart + yInverseFloatPart) / 2) );
            yValues[0] = ( yComp * ((xInverseFloatPart + yInverseFloatPart) / 2) );
            xValues[1] = ( xComp * ((xInverseFloatPart + yFloatPart) / 2) );
            yValues[1] = ( yComp * ((xInverseFloatPart + yFloatPart) / 2) );
            xValues[2] = ( xComp * ((xFloatPart + yInverseFloatPart) / 2) );
            yValues[2] = ( yComp * ((xFloatPart + yInverseFloatPart) / 2) );
            xValues[3] = ( xComp * ((xFloatPart + yFloatPart) / 2) );
            yValues[3] = ( yComp * ((xFloatPart + yFloatPart) / 2) );

            for (int i = 0; i < 4; ++i) {
                xVelocityTemp[ indexes[i] ] =
                        ( xVelocityTemp[ indexes[i] ] + xValues[i] ) / 2;
                yVelocityTemp[ indexes[i] ] =
                        ( yVelocityTemp[ indexes[i] ] + yValues[i] ) / 2;
            }


        }
    }
    xVelocity=xVelocityTemp;
    yVelocity=yVelocityTemp;
}
