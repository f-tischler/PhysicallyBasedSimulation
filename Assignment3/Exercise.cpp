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
#include <array>


static int xRes_static = -1;
constexpr double h=3.0f;
constexpr double h2=9.0f;

int index(Vector2 pos)
{
    return static_cast<int>(std::floor(pos.y() * xRes_static + pos.x()));
}

int index(const int x, const int y)
{
    return y * xRes_static + x;
}

void set_point(double* field, const Vector2 pos, const double value)
{
    field[index(pos)] = value;
}

void set_point(double *field, const int x, const int y, const double value)
{
    field[index(x, y)] = value;
}

double mix(const double x, const double y, const double alpha)
{
    return x * (1.0f - alpha) + y * alpha;
}

double fastModf(double x, double &part)
{
    part = floor(x);
    return x - part;
}

double sampleTrilinear(double *field, double x, double y) {
    
    double x_int_part = 0;
    double y_int_part = 0;

    const auto x_float_part = fastModf(x, x_int_part);
    const auto y_float_part = fastModf(y, y_int_part);

    const auto tmp1 = field[index({ x_int_part, y_int_part})];
    const auto tmp2 = field[index({ x_int_part + 1, y_int_part})];
    const auto tmp3 = field[index({ x_int_part, y_int_part + 1 })];
    const auto tmp4 = field[index({ x_int_part + 1, y_int_part + 1})];

    const auto tmp12 = mix(tmp1, tmp2, x_float_part);
    const auto tmp34 = mix(tmp3, tmp4, x_float_part);

    return mix(tmp12, tmp34, y_float_part);
}

void AdvectWithSemiLagrange(int xRes, int yRes, double dt,
                            double *xVelocity, double *yVelocity,
                            double *field, double* tempField)
{
    if(xRes_static == -1)
        xRes_static = xRes;

    for (auto y = 1; y < yRes - 1; ++y) 
    {
        for (auto x = 1; x < xRes - 1; ++x)
        {
            const Vector2 cur_cell(x, y);

            const auto x_comp = xVelocity[index(cur_cell)];
            const auto y_comp = yVelocity[index(cur_cell)];

            auto x_offset = x - x_comp * dt;
            auto y_offset = y - y_comp * dt;

            x_offset = std::min(xRes - 2.0, std::max(1.0, x_offset));
            y_offset = std::min(yRes - 2.0, std::max(1.0, y_offset));

            const auto new_value = sampleTrilinear(field, x_offset, y_offset);

            set_point(tempField, x, y, new_value);
        }
    }

}


void SolvePoisson(int xRes, int yRes, int iterations, double accuracy,
                  double* pressure, double* divergence)
{
    auto iteration = 0;
    auto error = 0.0;

    while (true)
    {
        for (auto y = 1; y < yRes - 1; ++y) 
        {
            for (auto x = 1; x < xRes - 1; ++x) 
            {
                const Vector2 cur_cell(
                {
                    static_cast<double>(x),
                    static_cast<double>(y)
                });

                const auto current = pressure[index(x, y)];

                //Using five points as it is necessary to take the local value into account as well
                const auto new_value = ( current +
                                    pressure[index(x + 1, y)] +
                                    pressure[index(x, y + 1)] +
                                    pressure[index(x - 1, y)] +
                                    pressure[index(x, y - 1)]
                                  ) / 5.0f;

                set_point(pressure, cur_cell, new_value);

                error += std::abs(current - new_value);
            }
        }

        ++iteration;
        
        if (error < accuracy || iteration >= iterations)
            break;
    }
}

void CorrectVelocities(int xRes, int yRes, double dt, const double* pressure,
                       double* xVelocity, double* yVelocity)
{
    for (auto y = 1; y < yRes - 1; ++y)
    {
        for (auto x = 1; x < xRes - 1; ++x) 
        {
            const auto presx2_y = pressure[index(x + 1,y)];
            const auto presxy2 = pressure[index(x,y + 1)];
            const auto presx1_y = pressure[index(x - 1,y)];
            const auto presxy1 = pressure[index(x,y - 1)];

            const auto presx = presx2_y-presx1_y;
            const auto presy = presxy2-presxy1;

            const auto deltax_vel = dt / 1.0f * (1.0f / h) * presx;
            const auto deltay_vel = dt / 1.0f * (1.0f / h) * presy;

            xVelocity[index(x,y)] = xVelocity[index(x,y)] - deltax_vel;
            yVelocity[index(x,y)] = yVelocity[index(x,y)] - deltay_vel;
        }
    }
}
