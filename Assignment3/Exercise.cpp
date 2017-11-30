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

void AdvectWithSemiLagrange(int xRes, int yRes, double dt, 
                            double *xVelocity, double *yVelocity, 
                            double *field, double* tempField)
{
    // Task 1
}


void SolvePoisson(int xRes, int yRes, int iterations, double accuracy, 
                  double* pressure, double* divergence)
{
    // Task 2
}

void CorrectVelocities(int xRes, int yRes, double dt, const double* pressure, 
                       double* xVelocity, double* yVelocity)
{
    // Task 3  
}

