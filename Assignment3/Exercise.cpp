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

int index(Vector2 pos)
{
    return static_cast<int>(std::floor( pos.y() * xRes_static + pos.x()));
}

int index(const int x, const int y)
{
    return y * xRes_static + x;
}

void set_point(double* field, const Vector2 pos, const double value)
{
    field[index(pos)] = value;
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

            set_point(tempField, cur_cell, new_value);
        }
    }

}


void SolvePoisson(int xRes, int yRes, int iterations, double accuracy,
                  double* pressure, double* divergence)
{
    auto temp_pressure = std::vector<double>(xRes * yRes, 0);

    while (true)
    {
        auto error = 0.0;

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

                set_point(temp_pressure.data(), cur_cell, new_value);

                error += std::abs(current - new_value);
            }
        }

        if(error < accuracy)
            break;

        for (auto y = 1; y < yRes - 1; ++y)
        {
            for (auto x = 1; x < xRes - 1; ++x)
            {
                const Vector2 cur_cell(
                {
                    static_cast<double>(x),
                    static_cast<double>(y)
                });

                const auto current = pressure[index(cur_cell)];

                //Using five points as it is necessary to take the local value into account as well
                const auto new_value = ( current +
                                    temp_pressure[index(x + 1, y)] +
                                    temp_pressure[index(x, y + 1)] +
                                    temp_pressure[index(x - 1, y)] +
                                    temp_pressure[index(x, y - 1)] -
                                    divergence[index(x, y)]
                                  ) / 5.0f;

                set_point(pressure, cur_cell, new_value);

                error += std::abs(current - new_value);
            }
        }

        if(error < accuracy)
            break;
    }
}

void CorrectVelocities(int xRes, int yRes, double dt, const double* pressure,
                       double* xVelocity, double* yVelocity)
{
    const auto resolution = yRes * xRes;

    auto x_velocity_temp = std::vector<double>(resolution);
    auto y_velocity_temp = std::vector<double>(resolution);

    for (auto y = 0; y < resolution; ++y) 
    {
        x_velocity_temp[y] = xVelocity[y] * 0.9f;
        y_velocity_temp[y] = yVelocity[y] * 0.9f;
    }

    for (auto y = 0; y < yRes; ++y) 
    {
        for (auto x = 0; x < xRes; ++x) 
        {
            const Vector2 cur_cell(x, y);

            const auto x_comp = xVelocity[index(cur_cell)];
            const auto y_comp = yVelocity[index(cur_cell)];

            auto x_offset = x + x_comp * dt;
            auto y_offset = y + y_comp * dt;

            x_offset = std::min(xRes - 2.0, std::max(1.0, x_offset));
            y_offset = std::min(yRes - 2.0, std::max(1.0, y_offset));

            double x_int_part, y_int_part;

            const auto x_float_part = fastModf(x_offset,x_int_part);
            const auto y_float_part = fastModf(y_offset,y_int_part);

            const auto x_inverse_float_part = 1 - x_float_part;
            const auto y_inverse_float_part = 1 - y_float_part;

            std::array<int, 4> indexes =
            {
                index({ x_int_part, y_int_part }),
                index({ x_int_part, y_int_part + 1 }),
                index({ x_int_part + 1, y_int_part }),
                index({ x_int_part + 1, y_int_part + 1 })
            };

            std::array<double, 4> x_values = 
            {
                x_comp * ((x_inverse_float_part + y_inverse_float_part) / 2),
                x_comp * ((x_inverse_float_part + y_float_part) / 2),
                x_comp * ((x_float_part + y_inverse_float_part) / 2),
                x_comp * ((x_float_part + y_float_part) / 2)
            };

            std::array<double, 4> y_values =
            {
                y_comp * ((x_inverse_float_part + y_inverse_float_part) / 2),
                y_comp * ((x_inverse_float_part + y_float_part) / 2),
                y_comp * ((x_float_part + y_inverse_float_part) / 2),
                y_comp * ((x_float_part + y_float_part) / 2)
            };

            for (auto i = 0; i < 4; ++i) 
            {
                x_velocity_temp[ indexes[i] ] =
                        ( x_velocity_temp[ indexes[i] ] + x_values[i] ) / 2;

                y_velocity_temp[ indexes[i] ] =
                        ( y_velocity_temp[ indexes[i] ] + y_values[i] ) / 2;
            }


        }
    }

    std::copy(x_velocity_temp.begin(), x_velocity_temp.end(), xVelocity);
    std::copy(y_velocity_temp.begin(), y_velocity_temp.end(), yVelocity);

    //xVelocity=x_velocity_temp;
    //yVelocity=y_velocity_temp;
}
