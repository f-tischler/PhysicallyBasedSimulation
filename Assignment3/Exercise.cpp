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


#include "Vec2.h"
#include <math.h>
#include <algorithm>
#include <cmath>
#include "gsl/gsl"


static int xRes_static = -1;
constexpr double h=3.0f;
constexpr double h2=9.0f;

template<typename T>
gsl::span<T> create_view(T*& first, const size_t size)
{
    const auto span = gsl::span<T>(first, size);

    first = nullptr;

    return span;
}

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

void set_point(const gsl::span<double> field, const int x, const int y, const double value)
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

double sampleTrilinear(const gsl::span<double> field, double x, double y)
{
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
    const auto resolution = xRes * yRes;

    const auto x_velocity_view = create_view(xVelocity, resolution);
    const auto y_velocity_view = create_view(yVelocity, resolution);
    const auto temp_field_view = create_view(tempField, resolution);
    const auto field_view = create_view(field, resolution);

    if(xRes_static == -1)
        xRes_static = xRes;

    for (auto y = 1; y < yRes - 1; ++y) 
    {
        for (auto x = 1; x < xRes - 1; ++x)
        {
            const auto velocity_x = x_velocity_view[index(x, y)];
            const auto velocity_y = y_velocity_view[index(x, y)];

            auto prev_x = gsl::narrow<int>(std::floor(x - velocity_x * dt));
            auto prev_y = gsl::narrow<int>(std::floor(y - velocity_y * dt));

            Ensures(prev_x > 0 && prev_x < xRes - 1);
            Ensures(prev_y > 0 && prev_y < yRes - 1);

            const auto new_value = sampleTrilinear(field_view, prev_x, prev_y);

            set_point(temp_field_view, x, y, new_value);
        }
    }

}


void SolvePoisson(int xRes, int yRes, int iterations, double accuracy,
                  double* pressure, double* divergence)
{
    const auto resolution = xRes * yRes;
    const auto pressure_view = create_view(pressure, resolution);
    //const auto divergence_view = create_view(divergence, resolution);

    auto iteration = 0;
    auto error = 0.0;

    while (true)
    {
        for (auto y = 1; y < yRes - 1; ++y) 
        {
            for (auto x = 1; x < xRes - 1; ++x) 
            {
                const auto current = pressure_view[index(x, y)];

                //Using five points as it is necessary to take the local value into account as well
                const auto new_value = ( current +
                    pressure_view[index(x + 1, y)] +
                    pressure_view[index(x, y + 1)] +
                    pressure_view[index(x - 1, y)] +
                    pressure_view[index(x, y - 1)]
                                  ) / 5.0f;

                set_point(pressure_view, x, y, new_value);

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
    const auto resolution = xRes * yRes;

    const auto x_velocity_view = create_view(xVelocity, resolution);
    const auto y_velocity_view = create_view(yVelocity, resolution);
    const auto pressure_view = create_view(pressure, resolution);

    for (auto y = 1; y < yRes - 1; ++y)
    {
        for (auto x = 1; x < xRes - 1; ++x) 
        {
            const auto presx2_y = pressure_view[index(x + 1,y)];
            const auto presxy2 = pressure_view[index(x,y + 1)];
            const auto presx1_y = pressure_view[index(x - 1,y)];
            const auto presxy1 = pressure_view[index(x,y - 1)];

            const auto presx = presx2_y-presx1_y;
            const auto presy = presxy2-presxy1;

            const auto deltax_vel = dt / 1.0f * (1.0f / h) * presx;
            const auto deltay_vel = dt / 1.0f * (1.0f / h) * presy;

            x_velocity_view[index(x,y)] = x_velocity_view[index(x,y)] - deltax_vel;
            y_velocity_view[index(x,y)] = y_velocity_view[index(x,y)] - deltay_vel;
        }
    }
}
