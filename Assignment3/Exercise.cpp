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

#define skip_source(x, y) if(x >= 90 && x < 110 && y >= 20 && y < 30) continue;
#define copy_source(x, y, src, dst) if(x >= 90 && x < 110 && y >= 20 && y < 30) dst[index(x,y)]=src[index(x,y)];


static int xRes_static = -1;

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

double sampleBilinear(const gsl::span<double> field,
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
    const auto resolution = xRes * yRes;

    const auto x_velocity_view = create_view(xVelocity, resolution);
    const auto y_velocity_view = create_view(yVelocity, resolution);
    const auto temp_field_view = create_view(tempField, resolution);
    const auto field_view = create_view(field, resolution);

    if(xRes_static == -1)
        xRes_static = xRes;
#pragma omp parallel for
    for (auto y = 0; y < yRes - 0; ++y)
    {
        for (auto x = 0; x < xRes - 0; ++x)
        {
            copy_source(x,y, field_view,temp_field_view)

            const auto velocity_x = x_velocity_view[index(x, y)];
            const auto velocity_y = y_velocity_view[index(x, y)];

            auto prev_x = x - velocity_x * dt * xRes;
            auto prev_y = y - velocity_y * dt * yRes;

            prev_x = std::max(0.0, std::min(xRes - 1.0, prev_x));
            prev_y = std::max(0.0, std::min(yRes - 1.0, prev_y));

            const auto new_value = sampleBilinear(field_view, prev_x, prev_y, xRes, yRes);

            set_point(temp_field_view, x, y, new_value);
        }
    }
}

void SolvePoisson(int xRes, int yRes, int iterations, double accuracy,
                  double* pressure, double* divergence)
{
    const auto resolution = xRes * yRes;
    const auto pressure_view = create_view(pressure, resolution);
    const auto divergence_view = create_view(divergence, resolution);

    std::vector<double> pressure_temp_view(pressure_view.begin(), pressure_view.end());

    const auto accuracy_sq = accuracy * accuracy;

    auto it = 0;
    auto error_sq = accuracy * accuracy;

    for (; it < iterations && error_sq >= accuracy_sq; ++it)
    {
        error_sq = 0.0;

        const auto read_view = gsl::span<double>(it % 2 == 0
            ? pressure_temp_view
            : pressure_view);

        const auto write_view = gsl::span<double>(it % 2 == 0
            ? pressure_view
            : pressure_temp_view);
#pragma omp parallel for reduction(+:error_sq)
        for (auto y = 1; y < yRes - 1; ++y) 
        {
            for (auto x = 1; x < xRes - 1; ++x) 
            {
                //Using five points as it is necessary to take the local value into account as well
                const auto new_value = (1.0 / resolution * divergence_view[index(x,y)] +
                                        read_view[index(x + 1, y)] +
                                        read_view[index(x - 1, y)] +
                                        read_view[index(x, y + 1)] +
                                        read_view[index(x, y - 1)]) / 4.0;

                set_point(write_view, x, y, new_value);

                const auto r = 1.0 / (xRes * yRes) * divergence_view[index(x, y)]
                    + read_view[index(x, y - 1)]
                    + read_view[index(x - 1, y)]
                    - 4 * read_view[index(x, y)]
                    + read_view[index(x, y + 1)]
                    + read_view[index(x + 1, y)];

                error_sq += r * r;
            }
        }
    }

    if (it % 2 != 0) return;
    
    std::copy(pressure_temp_view.begin(), pressure_temp_view.end(), pressure_view.begin());
}

void CorrectVelocities(int xRes, int yRes, double dt, const double* pressure,
                       double* xVelocity, double* yVelocity)
{
    const auto resolution = xRes * yRes;

    const auto x_velocity_view = create_view(xVelocity, resolution);
    const auto y_velocity_view = create_view(yVelocity, resolution);
    const auto pressure_view = create_view(pressure, resolution);

#pragma omp parallel for
    for (auto y = 1; y < yRes - 1; ++y)
    {
        for (auto x = 1; x < xRes - 1; ++x)
        {
            skip_source(x,y)
            const auto right = pressure_view[index(x + 1, y)];
            const auto mid = pressure_view[index(x, y)];
            const auto left = pressure_view[index(x - 1, y)];
            const auto top = pressure_view[index(x, y - 1)];
            const auto bottom = pressure_view[index(x, y + 1)];

            const auto x_diff = right - left;
            const auto y_diff = bottom - top;

            const auto deltax_vel = xRes * dt * 0.5 * x_diff;
            const auto deltay_vel = yRes * dt * 0.5 * y_diff;

            x_velocity_view[index(x,y)] -= deltax_vel;
            y_velocity_view[index(x,y)] -= deltay_vel;
        }
    }

}
