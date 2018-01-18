/******************************************************************
*
* Fluid2D.h
*
* Description: Class definition of 2D Euler flow scene & solver
*
* Physically-Based Simulation Proseminar WS 2016
*
* Interactive Graphics and Simulation Group
* Institute of Computer Science
* University of Innsbruck
*
*******************************************************************/

#ifndef __FLUID_2D_H__
#define __FLUID_2D_H__

#include <cmath>
#include <vector>
#include <iostream>
#include <cstring>


using namespace std;

const double solverAccuracy = 1e-5;
const int solverIterations = 1000;


class Fluid_2D  
{
public:
    Fluid_2D(int xRes, int yRes);
    virtual ~Fluid_2D();

    int get_xRes()         { return xRes; };
    int get_yRes()         { return yRes; };
    double* get_density()   { return density; };

    int iterations;  
    double accuracy; 

    void addDensity(double xMin, double xMax, double yMin, double yMax);
    void clearDensity();
    void toggleBoundaryCond()   { bndryCond = !bndryCond; };
    void toggleVorticity()   { addVort = !addVort; };

    void step();

protected:
    int xRes;               /* x resolution of cell grid */
    int yRes;               /* y resolution of cell grid */
    int totalCells;         /* Total number of cells */
    double dt;              /* Simulation time step size */
    int totalSteps;         /* Total timesteps taken */

    int bndryCond;          /* Toggle for boundary condition */
    int addVort;            /* Toggle for injecting turbulence */

    double* density;        /* Current density field */
    double* densityTemp;    /* Previous density field */
    double* pressure;       /* Pressure field */
    double* xVelocity;      /* Current x velocity component */
    double* xVelocityTemp;  /* Previous x velocity component */
    double* yVelocity;      /* Current y velocity component */
    double* yVelocityTemp;  /* Previous y velocity component */
    double* xForce;         /* x force component */
    double* yForce;         /* y force component */
    double* divergence;     /* Velocity divergence */
    double* curl;           /* Velocity curl */
    double* xCurlGrad;      /* x curl gradient component */ 
    double* yCurlGrad;      /* y curl gradient component */

    virtual void addForce();
    void addBuoyancy();
    void injectVorticity();
    void computeDivergence();
    void advectValues();
    void clearForce();
    void copyFields();
    void solvePressure();

    void setNeumannX(double* field);
    void setNeumannY(double* field);
    void setZeroX(double* field);
    void setZeroY(double* field);

    void copyBorderX(double* field);
    void copyBorderY(double* field);
};

#endif
