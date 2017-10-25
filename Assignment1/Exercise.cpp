/******************************************************************
*
* Exercise.cpp  
*
* Description: In this file - in the function TimeStep() - the various 
* numerical solvers of the first programming assignment have to be
* implemented. Feel free to add new local functions into this file. 
* Changes to other source files of the framework should not be 
* required. It is acceptable to assume knowledge about the scene
* topology (i.e. how springs and points are connected).
*
* Physically-Based Simulation Proseminar WS 2015
* 
* Interactive Graphics and Simulation Group
* Institute of Computer Science
* University of Innsbruck
*
*******************************************************************/

/* Standard includes */
#include <vector>
#include <random>
#include <cassert>
#include <fstream>
#include <array>

using namespace std;

/* Local includes */
#include "Vec2.h"
#include "Scene.h"

auto& get_time()
{
    static auto t = 0.0;

    return t;
}

auto& update_time(const double dt)
{
    return get_time() += dt;
}

void print_headers(ostream& os)
{
    static constexpr array<char*, 2> headers =
    {
        "t,", "rms"
    };
    
    for(const auto header : headers)
    {
        os << header;
    }

    os << "\n";
}

ostream& get_stream()
{
    static fstream stream = []()
    {
        fstream s("./lastrun.log", ios_base::out);

        const auto loc = locale("en-US");

        s.imbue(loc);

        print_headers(s);

        return s;
    }();

    return stream;
}

void print_value()
{
    get_stream() << endl;
}

template<class T>
void print_value(const T& value)
{
    get_stream() << value;
}

template<class T, class...Ts>
void print(const T& value, Ts&&...rest)
{
    print_value(value);

    if constexpr(sizeof...(rest) > 0)
    {
        print_value(",");
        print(std::forward<Ts>(rest)...);
    }
    else
    {
        print_value();
    }
}

Point& get_other_spring_end(const Point& point, const Spring& spring)
{
    return spring.getPoint(0) == &point
        ? *spring.getPoint(1)
        : *spring.getPoint(0);
}

Vec2 compute_internal_forces(const Point& point, const vector<Spring>& springs)
{
	auto force = Vec2(0.0, 0.0);

	for (auto& spring : springs)
	{
		if (spring.getPoint(0) != &point &&
			spring.getPoint(1) != &point)
		{
			continue;
		}

		const auto spring_point = spring.getPoint(0) == &point
			? spring.getPoint(1)
			: spring.getPoint(0);

		const auto connection =
			point.getPos() - spring_point->getPos();

		const auto distance = connection.length();

		if (abs(distance) < 0.00000001)
			continue;

		const auto direction = connection.normalize();

		force += spring.getStiffness() *
			(spring.getRestLength() - distance) * direction;
	}

	return force;
}

Vec2 compute_acceleration(const Point& point)
{
	return (point.getForce() - point.getDamping() * point.getVel()) /
		point.getMass();
}

void update_forces(Point& point, const vector<Spring>& springs)
{
	// internal forces
	point.setForce(compute_internal_forces(point, springs));

	// external forces
	point.addForce(point.getUserForce());
}

Vec2 compute_acceleration(Point& point, const vector<Spring>& springs)
{
	update_forces(point, springs);
	return compute_acceleration(point);
}

void apply_external_forces(Point& point, const bool interaction)
{
    static default_random_engine rng;
    static uniform_real_distribution<> rnd(-50, 50);

    // gravity
    static constexpr auto g = -10;

    point.setUserForce(Vec2(0, point.getMass() * g));

    if (interaction)
    {
        point.setUserForce(point.getUserForce() + Vec2(
            rnd(rng), abs(rnd(rng))));
    }
}

template<class F>
void apply_method(vector<Point>& points, 
    const bool interaction, 
    const F& method)
{
    for (auto& point : points)
    {
        if (point.isFixed())
            continue;

        apply_external_forces(point, interaction);

        method(point);
    }
}

void analytical(const double dt,
                vector<Point>& points,
                const vector<Spring>& springs)
            {
                static constexpr auto g = -10.0;

                const auto t = get_time();

                apply_method(points, false, [&](auto& point)
                {
                    const auto m = point.getMass();
                    const auto d = point.getDamping();

                    const auto wr = d / (2 * m);

                    for (const auto& spring : springs)
                    {
                        if (spring.getPoint(0) != &point &&
                            spring.getPoint(1) != &point)
                        {
                            continue;
                        }

                        const auto& spring_point = get_other_spring_end(point, spring);

                        const auto connection =
                            spring_point.getPos() - point.getPos();

                        const auto direction = connection.normalize();

                        const auto actual_gravity = g * abs(direction.y);

                        const auto l = spring.getRestLength();
                        const auto k = spring.getStiffness();

                        const auto x0 = -l;

                        const auto w = sqrt(k / m);

                        // ensure under-damping
                        assert(wr * wr  < w * w &&
                            point.getDamping() < 2 * sqrt(m * spring.getStiffness()));

                        const auto wbar = sqrt(w * w - wr * wr);

                        const auto a = m * actual_gravity / k;
                        const auto b = a * (wr / wbar);

                        // update position --------------------------------------------------------------
                        const auto x = exp(-wr * t) * (a*cos(wbar * t) + b *sin(wbar * t)) -
                            m * actual_gravity / k + x0;

                        point.setPos(spring_point.getPos() + x * direction);
                    }
                });
            }

void compare(const vector<Point>& expected, const vector<Point>& actual)
{
    assert(expected.size() == actual.size());

    auto pos_error = 0.0;

    for(auto i = 0u; i < expected.size(); i++)
    {
        pos_error += (expected[i].getPos() - actual[i].getPos()).length_sq();
    }

    print(get_time(), sqrt(pos_error / expected.size()));
}

template<bool Compare, class F>
void apply_method(const double dt,
                  vector<Point>& points,
                  const vector<Spring>& springs,
                  const bool interaction,
                  const F& method)
{
    if constexpr (Compare)
    {
        static auto reference_points = points;

        apply_method(points, false, method);

        analytical(dt, reference_points, springs);

        compare(reference_points, points);
    }
    else
    {
        apply_method(points, interaction, method);
    }
}

template<bool Compare>
void euler(const double dt,
           vector<Point>& points,
           const vector<Spring>& springs,
           const bool interaction)
{
    apply_method<Compare>(dt, points, springs, interaction, 
        [&](auto& point)
    {
        // x(t + h) = x(t) + h * v(t)
        // v(t + h) = v(t) + h * a(t)

        const auto new_position = point.getPos() + point.getVel() * dt;
        const auto a = compute_acceleration(point, springs);

        point.setPos(new_position);
        point.setVel(point.getVel() + a * dt);
    });
}

template<bool Compare>
void symplectic(const double dt,
				vector<Point>& points,
                const vector<Spring>& springs,
				const bool interaction)
{
    apply_method<Compare>(dt, points, springs, interaction,
        [&](auto& point)
    {
        // x(t + h) = x(t) + h * v(t)
        // v(t + h) = v(t) + h * a(t + h)

        point.setPos(point.getPos() + point.getVel() * dt);

        const auto a = compute_acceleration(point, springs);

        point.setVel(point.getVel() + a * dt);
    });
}

template<bool Compare>
void midpoint(const double dt, 
			  vector<Point>& points, 
              const vector<Spring>& springs,
	          const bool interaction)
{
    apply_method<Compare>(dt, points, springs, interaction,
        [&](auto& point)
    {
        const auto a = compute_acceleration(point, springs);

        const auto original_velocity = point.getVel();

        point.setVel(point.getVel() + dt / 2.0 * a);

        const auto original_position = point.getPos();

        point.setPos(point.getPos() + dt / 2.0 * point.getVel());

        const auto a_new = compute_acceleration(point, springs);

        point.setPos(original_position + dt * point.getVel());

        point.setVel(original_velocity + dt * a_new);
    });
}

template<bool Compare>
void leapfrog(const double dt, 
			  vector<Point>& points, 
			  vector<Spring>& springs, 
	          const bool interaction)
{
    apply_method<Compare>(dt, points, springs, interaction,
        [&](auto& point)
    {
        const auto a = compute_acceleration(point, springs);

        const auto old_velocity = point.getVel() - dt / 2.0 * a;

        const auto new_velocity = old_velocity + dt * a;

        point.setPos(point.getPos() + dt  * new_velocity);

        point.setVel(new_velocity);
    });
}

/******************************************************************
*
* TimeStep
*
* This function is called every time step of the dynamic simulation.
* Positions, velocities, etc. should be updated to simulate motion
* of the mass points of the 2D scene. The selected solver is passed
* to the function as well as the time step, the springs, and the
* mass points.
*
*******************************************************************/

void TimeStep(const double dt, const Scene::Method method,
               vector<Point>& points, vector<Spring>& springs, const bool interaction)
{
    update_time(dt);

	switch (method)
	{
		case Scene::EULER:
		{
			return euler<true>(dt, points, springs, interaction);
		}

		case Scene::SYMPLECTIC:
		{
			return symplectic<true>(dt, points, springs, interaction);
		}

		case Scene::LEAPFROG:
		{
            return leapfrog<true>(dt, points, springs, interaction);
		}

		case Scene::MIDPOINT:
		{
			return midpoint<true>(dt, points, springs, interaction);
		}
	}
}
