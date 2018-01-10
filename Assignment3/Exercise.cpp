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

#include <cmath>
#include <algorithm>
#include "gsl/gsl"

static int xRes_static = -1;

int index(const int x, const int y)
{
    return y * xRes_static + x;
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

double sampleBilinear(double* field,
                      const double x,
                      const double y,
                      const int width,
                      const int height)
{
    double x_int_part = 0;
    double y_int_part = 0;

    const auto x_float_part = fastModf(x, x_int_part);
    const auto y_float_part = fastModf(y, y_int_part);

    const auto index_x = gsl::narrow<int>(x_int_part);
    const auto index_y = gsl::narrow<int>(y_int_part);

    const auto index_neighbour_x = std::max(0, std::min(width - 1,  index_x + 1));
    const auto index_neighbour_y = std::max(0, std::min(height - 1, index_y + 1));

    const auto tmp1 = field[index(index_x, index_y)];
    const auto tmp2 = field[index(index_neighbour_x, index_y)];
    const auto tmp3 = field[index(index_x, index_neighbour_y)];
    const auto tmp4 = field[index(index_neighbour_x, index_neighbour_y)];
        
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

    #pragma omp parallel for
    for (auto y = 0; y < yRes - 0; ++y)
    {
        for (auto x = 0; x < xRes - 0; ++x)
        {
            const auto velocity_x = xVelocity[index(x, y)];
            const auto velocity_y = yVelocity[index(x, y)];

            auto prev_x = x - velocity_x * dt * xRes;
            auto prev_y = y - velocity_y * dt * yRes;

            prev_x = std::max(0.0, std::min(xRes - 1.0, prev_x));
            prev_y = std::max(0.0, std::min(yRes - 1.0, prev_y));

            const auto new_value = sampleBilinear(field, prev_x, prev_y, xRes, yRes);

            set_point(tempField, x, y, new_value);
        }
    }
}

void SolvePoisson(int xRes, int yRes, int iterations, double accuracy,
                  double* pressure, double* divergence)
{
    const auto resolution = xRes * yRes;

    const auto accuracy_sq = accuracy * accuracy;

    auto it = 0;
    auto error_sq = accuracy * accuracy;

    for (; it < iterations && error_sq >= accuracy_sq; it+=2)
    {
        error_sq = 0.0;
#pragma omp parallel for reduction(+:error_sq)
        for (auto y = 1; y < yRes - 1; ++y)
        {
            for (auto x = 1; x < xRes - 1; ++x)
            {
                const auto new_value = (1.0 / resolution * divergence[index(x,y)] +
                        pressure[index(x + 1, y)] +
                        pressure[index(x - 1, y)] +
                        pressure[index(x, y + 1)] +
                        pressure[index(x, y - 1)]) / 4.0;


                const auto r = 1.0 / (xRes * yRes) * divergence[index(x, y)]
                               + pressure[index(x, y - 1)]
                               + pressure[index(x - 1, y)]
                               - 4 * pressure[index(x, y)]
                               + pressure[index(x, y + 1)]
                               + pressure[index(x + 1, y)];

                set_point(pressure, x, y, new_value);

                error_sq += r * r;
            }
        }
    }

}

void CorrectVelocities(int xRes, int yRes, double dt, const double* pressure,
                       double* xVelocity, double* yVelocity)
{
    #pragma omp parallel for
    for (auto y = 1; y < yRes - 1; ++y)
    {
        for (auto x = 1; x < xRes - 1; ++x)
        {
            const auto right = pressure[index(x + 1, y)];
            const auto left = pressure[index(x - 1, y)];
            const auto top = pressure[index(x, y - 1)];
            const auto bottom = pressure[index(x, y + 1)];

            const auto x_diff = right - left;
            const auto y_diff = bottom - top;

            const auto deltax_vel = xRes * dt * 0.5 * x_diff;
            const auto deltay_vel = yRes * dt * 0.5 * y_diff;

            xVelocity[index(x,y)] -= deltax_vel;
            yVelocity[index(x,y)] -= deltay_vel;
        }
    }

}
