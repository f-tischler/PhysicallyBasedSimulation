/******************************************************************               
*                                                                                 
* Fluid2D.cpp                                                                       
*                                                                                 
* Description: Implementation of functions for setting up 2D Euler 
* flow scene & solving the underlying differential equations via
* operator splitting; semi-Lagrangian advection is employed, the
* pressures are obtained via Gauss-Seidel iteration; artificial
* turbulence is added following Fedkiw et al. (vorticity confinement)
*                                                                                 
* Physically-Based Simulation Proseminar WS 2016                                  
*                                                                                 
* Interactive Graphics and Simulation Group                                       
* Institute of Computer Science                                                   
* University of Innsbruck                                                         
*                                                                                 
*******************************************************************/ 

/* Standard includes */
#include <sstream>
#include <stdlib.h>

/* Local includes */
#include "Fluid2D.h"


/* External functions for implementing advection, the pressure
   solver, and the correction of velocities */
extern void AdvectWithSemiLagrange(int xRes, int yRes, double dt, 
                                   double* xVelocity, double* yVelocity, 
                                   double *field, double *tempField);

extern void SolvePoisson(int xRes, int yRes, int iterations, double accuracy, 
                         double* pressure, double* divergence);

extern void CorrectVelocities(int xRes, int yRes, double dt, const double* pressure, 
                              double* xVelocity, double* yVelocity); 


Fluid_2D::Fluid_2D(int _xRes, int _yRes) : xRes(_xRes), yRes(_yRes)
{
    dt = 0.1;             /* Time step */
    totalSteps = 0;      
    bndryCond = 0;        
    addVort = 0;

    iterations = solverIterations;
    accuracy = solverAccuracy;

    /* Allocate fields */
    totalCells = xRes * yRes;
    divergence = new double[totalCells];
    pressure = new double[totalCells];
    xVelocity = new double[totalCells];
    yVelocity = new double[totalCells];
    xVelocityTemp = new double[totalCells];
    yVelocityTemp = new double[totalCells];
    xForce = new double[totalCells];
    yForce = new double[totalCells];
    density = new double[totalCells];
    densityTemp = new double[totalCells];
    curl = new double[totalCells];
    xCurlGrad = new double[totalCells];
    yCurlGrad = new double[totalCells];

    /* Initialize fields */
    for (int i = 0; i < totalCells; i++)
    {
        divergence[i] = 0.0;
        pressure[i] = 0.0;
        xVelocity[i] = 0.0;
        yVelocity[i] = 0.0;
        xVelocityTemp[i] = 0.0;
        yVelocityTemp[i] = 0.0;
        xForce[i] = 0.0;
        yForce[i] = 0.0;
        density[i] = 0.0;
        densityTemp[i] = 0.0;
        curl[i] = 0.0;
        xCurlGrad[i] = 0.0;
        yCurlGrad[i] = 0.0;
    }
}

Fluid_2D::~Fluid_2D()
{
    if (density)  delete[] density;
    if (densityTemp)  delete[] densityTemp;
    if (divergence)  delete[] divergence;
    if (pressure)  delete[] pressure;
    if (xVelocity)  delete[] xVelocity;
    if (yVelocity)  delete[] yVelocity;
    if (xVelocityTemp)  delete[] xVelocityTemp;
    if (yVelocityTemp)  delete[] yVelocityTemp;
    if (xForce)  delete[] xForce;
    if (yForce)  delete[] yForce;
    if (curl)  delete[] curl;
    if (xCurlGrad)  delete[] xCurlGrad;
    if (yCurlGrad)  delete[] yCurlGrad;
}
  

/*----------------------------------------------------------------*/

void Fluid_2D::addDensity(double xMin, double xMax, double yMin, double yMax)
{
    /* Add density at fixed location */
    for (int y = (int)(yMin * yRes); y < (int)(yMax * yRes); y++)
        for (int x = (int)(xMin * xRes); x < (int)(xMax * xRes); x++)
        {
            const int index = y*xRes+x; 
            density[index] = 1.0;      
        }
}

void Fluid_2D::clearDensity()
{
    /* Clear density (everything else remains unchanged) */
    for (int i = 0; i < totalCells; i++) 
        density[i] = 0.0;      
}


/*------------------------------------------------------------------
| Time step for solving the Euler flow equations using operator
| splitting
------------------------------------------------------------------*/

void Fluid_2D::step()
{
    /* Body force terms */
    addBuoyancy();      /* Add lifting force proportional to density */

    if(addVort)         /* Inject artificial turbulence */
        injectVorticity();
    
    addForce();

    /* Advect densities as well as velocity */
    AdvectWithSemiLagrange(xRes, yRes, dt, xVelocity, yVelocity,
                           density, densityTemp);
    AdvectWithSemiLagrange(xRes, yRes, dt, xVelocity, yVelocity,
                           xVelocity, xVelocityTemp);
    AdvectWithSemiLagrange(xRes, yRes, dt, xVelocity, yVelocity,
                           yVelocity, yVelocityTemp);

    /* Copy/update advected fields */
    copyFields();

    /* Solve for pressure, ensuring divergence-free velocity field */
    solvePressure();
    
    /* Clear forces for next step */
    clearForce();
   
    totalSteps++;
}


/*----------------------------------------------------------------*/

void Fluid_2D::addBuoyancy()
{
    /* Lifting force proportional to density */
    for (int i = 0; i < totalCells; i++)
        yForce[i] += 0.002 * density[i];     
}

void Fluid_2D::injectVorticity()
{
    /* Inject turbulence via vorticity confinement */
    double epsilon = 0.15;
    const double idx = xRes / 2.0;
    const double dx = 1.0 / xRes;

    /* Compute curl of vector field */
    for (int y = 1; y < yRes-1; y++) 
        for (int x = 1; x < xRes-1; x++)
        {
            const int index = y*xRes + x;
            const double xCurl = (yVelocity[index + 1] - yVelocity[index -1]) * idx;
            const double yCurl = (xVelocity[index + xRes] - xVelocity[index - xRes]) * idx;
            curl[index] = (xCurl - yCurl); 
        }

    /* Compute curl gradient vector */
    for (int y = 1; y < yRes-1; y++) 
        for (int x = 1; x < xRes-1; x++)
        {
            const int index = y*xRes + x;
            xCurlGrad[index] = (fabs(curl[index + 1]) - fabs(curl[index -1])) * idx;
            yCurlGrad[index] = (fabs(curl[index + xRes]) - fabs(curl[index - xRes])) * idx;
            
            const double len = sqrt(xCurlGrad[index]*xCurlGrad[index] +
                                    yCurlGrad[index]*yCurlGrad[index]);

            /* Normalize length */
            if(fabs(len) > 0.000001)
            {
                xCurlGrad[index] /= len;
                yCurlGrad[index] /= len;
            }

            /* Add turbulence force to total body force */
            xForce[index] += epsilon * dx * curl[index] * yCurlGrad[index];
            yForce[index] -= epsilon * dx * curl[index] * xCurlGrad[index];              
        }
}

void Fluid_2D::addForce()
{
    for (int i = 0; i < totalCells; i++) 
    {
        xVelocity[i] += dt * xForce[i];
        yVelocity[i] += dt * yForce[i];
    }
}

/*----------------------------------------------------------------*/

void Fluid_2D::solvePressure()
{
    /* Set appropriate boundary condition - open vs. closed domain */
    if(bndryCond)
    {
        setZeroY(yVelocity);
        setZeroX(xVelocity);
    }
    else
    {  
        setNeumannX(xVelocity);
        setNeumannY(yVelocity);
        setZeroY(xVelocity);
        setZeroX(yVelocity);       
    }
    
    /* Compute velocity field divergence */
    computeDivergence();

    copyBorderX(pressure);
    copyBorderY(pressure);

    /* Solve for pressures and make field divergence-free */
    SolvePoisson(xRes, yRes, iterations, accuracy, pressure, divergence);  
    CorrectVelocities(xRes, yRes, dt, pressure, xVelocity, yVelocity);
}

void Fluid_2D::computeDivergence()
{
    const double dx = 1.0 / xRes;
    const double idtx = 1.0 / (2.0*(dt * dx));
 
    for (int y = 1; y < yRes-1; y++) 
        for (int x = 1; x < xRes-1; x++)
        {
            const int index = y*xRes + x;
            const double xComponent = (xVelocity[index + 1] - xVelocity[index -1]) * idtx;
            const double yComponent = (yVelocity[index + xRes] - yVelocity[index - xRes]) * idtx;
            divergence[index] = -(xComponent + yComponent); 
        }
}
    
void Fluid_2D::copyFields()
{
    int size = totalCells*sizeof(double);
    memcpy(density, densityTemp, size);
    memcpy(xVelocity, xVelocityTemp, size);
    memcpy(yVelocity, yVelocityTemp, size);
}
                                                      
void Fluid_2D::clearForce()
{
    for (int i = 0; i < totalCells; i++)
        xForce[i] = yForce[i] = 0.0;
}


/*----------------------------------------------------------------*/

void Fluid_2D::setNeumannX(double* field)
{
    for (int y = 0; y < yRes; y++)
    {
        int index = y * xRes;
        field[index] = field[index + 1];
        field[index + xRes - 1] = field[index + xRes - 2];
    }
}

void Fluid_2D::setNeumannY(double* field)
{
    for (int x = 0; x < xRes; x++)
    {
        int index = x;
        field[index] = field[index + xRes];

        index = x + (yRes - 1) * xRes;
        field[index] = field[index - xRes];
    }
}

void Fluid_2D::setZeroX(double* field)
{
    for (int y = 0; y < yRes; y++)
    {
        int index = y * xRes;
        field[index] = 0.0;
        field[index + xRes - 1] = 0.0;
    }
}

void Fluid_2D::setZeroY(double* field)
{
    for (int x = 0; x < xRes; x++)
    {
        int index = x;
        field[index] = 0.0;

        index = x + (yRes - 1) * xRes;
        field[index] = 0.0;
    }
}

void Fluid_2D::copyBorderX(double* field)
{
    for (int y = 0; y < yRes; y++)
    {
        int index = y * xRes;
        field[index] = field[index + 1];
        field[index + xRes - 1] = field[index + (xRes - 1) - 1];
    }
}

void Fluid_2D::copyBorderY(double* field)
{
    for (int x = 0; x < xRes; x++)
    {
        int index = x;
        field[index] = field[index + xRes];

        index = x + (yRes - 1) * xRes;
        field[index] = field[index - xRes];
    }
}
