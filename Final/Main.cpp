/******************************************************************
*
* Main.cpp
*
* Description: This is an implementation of a collision detection
* exmple.
*
* Physically-Based Simulation Proseminar WS 2016
*
* Interactive Graphics and Simulation Group
* Institute of Computer Science
* University of Innsbruck
*
*******************************************************************/

/* Standard includes */
#include <thread>
#include <random>
#include <iostream>
#include <cmath>
#include <iostream>
#include <vector>
#include <SFML/Graphics.hpp>

/* Local includes */
#include "Polygon.h"
#include "Console.hpp"
#include "ContactInfo.hpp"
#include "smoother.h"

enum class GameLoopType
{
	Fixed = 0,
	Variable,
	Unkown
};

/*----------------------------------------------------------------*/

void render(sf::RenderWindow& window, const std::vector<polygon>& polygons)
{
    window.clear();
    
    for (auto& polygon : polygons)
    {
        polygon.draw(window);
    }

	Console::instance().print(window);

    window.display();
}

bool lineSegmentIntersection(
	double Ax, double Ay,
	double Bx, double By,
	double Cx, double Cy,
	double Dx, double Dy,
	double& X, double& Y) {

	double  distAB, theCos, theSin, newX, ABpos;

	//  Fail if either line segment is zero-length.
	if (Ax == Bx && Ay == By || Cx == Dx && Cy == Dy) return false;

	//  Fail if the segments share an end-point.
	if (Ax == Cx && Ay == Cy || Bx == Cx && By == Cy
		|| Ax == Dx && Ay == Dy || Bx == Dx && By == Dy) {
		return false;
	}

	//  (1) Translate the system so that point A is on the origin.
	Bx -= Ax; By -= Ay;
	Cx -= Ax; Cy -= Ay;
	Dx -= Ax; Dy -= Ay;

	//  Discover the length of segment A-B.
	distAB = sqrt(Bx*Bx + By * By);

	//  (2) Rotate the system so that point B is on the positive X axis.
	theCos = Bx / distAB;
	theSin = By / distAB;
	newX = Cx * theCos + Cy * theSin;
	Cy = Cy * theCos - Cx * theSin; Cx = newX;
	newX = Dx * theCos + Dy * theSin;
	Dy = Dy * theCos - Dx * theSin; Dx = newX;

	//  Fail if segment C-D doesn't cross line A-B.
	if (Cy<0. && Dy<0. || Cy >= 0. && Dy >= 0.) return false;

	//  (3) Discover the position of the intersection point along line A-B.
	ABpos = Dx + (Cx - Dx)*Dy / (Dy - Cy);

	//  Fail if segment C-D crosses line A-B outside of segment A-B.
	if (ABpos<0. || ABpos>distAB) return false;

	//  (4) Apply the discovered position to line A-B in the original coordinate system.
	X = Ax + ABpos * theCos;
	Y = Ay + ABpos * theSin;

	//  Success.
	return true;
}

bool inside(const Vector2d point, 
    const Vector2d point_offset, 
    const std::vector<line_t>& lines, 
    const Vector2d line_offset)
{
    auto intersections = 0;

    for (const auto& line_b : lines)
    {
        const auto x1 = point_offset.x() + point.x();
        const auto y1 = point_offset.y() + point.y();

        const auto x2 = point_offset.x() + point.x() * 100000.0;
        const auto y2 = point_offset.y() + point.y() * 100000.0;

        const auto x3 = line_offset.x() + std::get<0>(std::get<0>(line_b)).x();
        const auto y3 = line_offset.y() + std::get<0>(std::get<0>(line_b)).y();

        const auto x4 = line_offset.x() + std::get<0>(std::get<1>(line_b)).x();
        const auto y4 = line_offset.y() + std::get<0>(std::get<1>(line_b)).y();

        double ix, iy;
        if (!lineSegmentIntersection(x1, y1, x2, y2, x3, y3, x4, y4, ix, iy))
            continue;

        ++intersections;
    }

    return intersections % 2 == 1;
}

void find_intersections(physical_object& object_a, physical_object& object_b, std::vector<contact_info>& contacts)
{
    const auto get_lines = [](const physical_object& object)
    {
        std::vector<line_t> lines;

        const auto& points = object.points();

        for (size_t i = 0; i < points.size(); i++)
        {
            auto end_index = i == points.size() - 1
                ? 0
                : i + 1;

            lines.emplace_back(points[i], points[end_index]);
        }

        return lines;
    };

    const auto points_a = object_a.points();
    const auto lines_b = get_lines(object_b);

    const auto position_a = object_a.position();
    const auto position_b = object_b.position();

    for (const auto& point_a : points_a)
    {
        const auto point = std::get<0>(point_a);

        if (!inside(point, position_a, lines_b, position_b))
            continue;

        line_t line;
        auto distance = std::numeric_limits<double>::max();
        auto penetration_depth = 0.0;
        Vector2d point_of_intersection;

        for (const auto& line_b : lines_b)
        {
            const auto line_start = position_b + std::get<0>(std::get<0>(line_b));
            const auto line_end = position_b + std::get<0>(std::get<1>(line_b));

            const auto x1 = position_a.x();
            const auto y1 = position_a.y();

            const auto x2 = position_a.x() + point.x();
            const auto y2 = position_a.y() + point.y();

            const auto x3 = line_start.x();
            const auto y3 = line_start.y();

            const auto x4 = line_end.x();
            const auto y4 = line_end.y();

            double ix, iy;
            if (!lineSegmentIntersection(x1, y1, x2, y2, x3, y3, x4, y4, ix, iy))
                continue;

            const auto current_distance = 
                (Vector2d(x2, y2)-Vector2d(ix, iy)).norm();

            if (current_distance >= distance)
                continue;

            distance = current_distance;
            line = line_b;
            penetration_depth = current_distance;
            point_of_intersection = Vector2d(ix, iy);
        }

        contacts.emplace_back(object_a, point_a, object_b, line, penetration_depth, point_of_intersection);
    }
}

bool intersects(physical_object& a, physical_object& b, std::vector<contact_info>& contacts)
{
    if ((a.center_of_mass_global() -
        b.center_of_mass_global()).norm() >
        a.bounding_radius() +
        b.bounding_radius())
        return false;

    find_intersections(a, b, contacts);
    find_intersections(b, a, contacts);

    return !contacts.empty();
}

std::vector<contact_info> collision_detection(std::vector<polygon>& polygons)
{
    for (auto& polygon_a : polygons)
    {
        polygon_a.clear_contacts();
    }

	std::vector<contact_info> global_contacts;
	for (auto i = 0u; i < polygons.size(); ++i)
	{
        auto& polygon_a = polygons[i];

		for (auto j = i + 1; j < polygons.size(); ++j)
		{
            auto& polygon_b = polygons[j];

            auto& object_a = polygon_a.get_physical_object();
            auto& object_b = polygon_b.get_physical_object();

			if (object_a.get_type() == object_type::fixed &&
                object_b.get_type() == object_type::fixed)
				continue;

            std::vector<contact_info> local_contacts;
            if (!intersects(object_a, object_b, local_contacts))
                continue;

            polygon_a.add_contacts(local_contacts);
			polygon_b.add_contacts(local_contacts);

            std::move(
                local_contacts.begin(), 
                local_contacts.end(), 
		        std::back_inserter(global_contacts));
		}
	}

	return global_contacts;
}

void collision_resolution(const std::vector<contact_info>& contacts)
{
    for (const auto& contact : contacts)
    {
        const auto line_start = std::get<0>(std::get<0>(contact.line));
        const auto line_end = std::get<0>(std::get<1>(contact.line));

        const auto normal = ((line_end + line_start) / 2.0).normalized();

        auto& a = contact.line_owner;
        auto& b = contact.point_owner;

        const auto b_point_offset = std::get<0>(contact.point);
        const auto b_point_velocity = std::get<1>(contact.point);

        const Vector2d a_point_offset =
            (std::get<0>(std::get<0>(contact.line)) +
                std::get<0>(std::get<1>(contact.line))) / 2.0;

        const Vector2d a_point_velocity =
            (std::get<1>(std::get<0>(contact.line)) +
                std::get<1>(std::get<1>(contact.line))) / 2.0;

        const Vector2d rv = b_point_velocity - a_point_velocity;
        const auto relative_velocity = rv.dot(normal);

        if (relative_velocity > 0)
            continue;

        const auto e = rv.squaredNorm() < (gravity * 0.016).squaredNorm() + 0.01
            ? 0
            : 0.5;

        const auto ra_n = cross2(a_point_offset, normal);
        const auto rb_n = cross2(b_point_offset, normal);

        const auto t_a = a.inverse_mass() + a.inverse_inertia() * ra_n * ra_n;
        const auto t_b = b.inverse_mass() + b.inverse_inertia() * rb_n * rb_n;
        const auto denom = t_a + t_b;

        const auto j = -(1 + e) * relative_velocity / denom;
        
        const auto normal_impulse = j * normal;

        a.apply_impulse(-normal_impulse, a_point_offset);
        b.apply_impulse( normal_impulse, b_point_offset);

        // friction
        const auto tangent = (rv - normal * rv.dot(normal)).normalized();

        const auto jt = -rv.dot(tangent) / denom;

        if (std::abs(jt) < 0.000001) continue;
        
        const auto static_friction = 0.61;
        const auto dynamic_friction = 0.47;

        const auto tangent_impulse = std::abs(jt) < j * static_friction
            ? (tangent * jt).eval()
            : (tangent * -j * dynamic_friction).eval();

        a.apply_impulse(-tangent_impulse, a_point_offset);
        b.apply_impulse(tangent_impulse, b_point_offset);
    }
}

void correct_positions(const std::vector<contact_info>& contacts)
{
    for (const auto& contact : contacts)
    {
        const auto line_start = std::get<0>(std::get<0>(contact.line));
        const auto line_end = std::get<0>(std::get<1>(contact.line));
        const auto normal = ((line_end + line_start) / 2.0).normalized();

        auto& a = contact.line_owner;
        auto& b = contact.point_owner;

        const auto percent = 0.2; // usually 20% to 80%
        const auto slop = 0.05; // usually 0.01 to 0.1

        const Vector2d correction = std::max(contact.penetration_depth - slop, 0.0)
            / (a.inverse_mass() + b.inverse_mass()) * percent * normal;

        a.move(-a.inverse_mass() * correction);
        b.move( b.inverse_mass() * correction);
    }
}


void update(std::vector<polygon>& polygons, const double dt)
{
	const auto contacts = collision_detection(polygons);
	
	collision_resolution(contacts);

	for (auto& polygon : polygons)
	{
		polygon.update(dt);
	}

    correct_positions(contacts);
}

int main()
{
    auto increase_polygon = false;
    auto polygon_vertex_count = 3;

    std::vector<polygon> polygons;

    double xs = 0;
    double ys = 0;

    constexpr auto width = 1280;
    constexpr auto height = 800;

	Console::instance().init();

    sf::RenderWindow window(sf::VideoMode(width, height), "2D Collision detection");

	polygons.emplace_back(polygon::create_rectangle(Vector2(400, 600), Vector2(500, 80)));
    polygons.emplace_back(polygon::create_rectangle(Vector2(400, 300), Vector2(500, 80)));

    using namespace std::chrono;
    using clock = high_resolution_clock;

	const auto fps_60 = duration<double>(16ms);
	const auto fps_30 = duration<double>(32ms);
    const auto interval = fps_60;


	auto game_loop_type = GameLoopType::Fixed;

	smoother<double, 10> draw_time_smooth;
	smoother<double, 10> update_time_smooth;

    auto last_time = clock::now();

	auto measure = [&](auto name, auto& smoother, auto func) {

		auto start_frame = clock::now();

		func();

		auto elapsed = duration_cast<milliseconds>(clock::now() - start_frame);
		smoother.add(elapsed.count());
		Console::instance().set_param(name, smoother.get());
	};

    std::default_random_engine rng;

    auto process_events = [&]()
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            switch (event.type)
            {
            case sf::Event::Closed: window.close(); break;

            case sf::Event::MouseMoved:
            {
                xs = sf::Mouse::getPosition().x - window.getPosition().x - 10;
                ys = sf::Mouse::getPosition().y - window.getPosition().y - 35;
            } break;

            case sf::Event::MouseButtonPressed:
            {
                polygons.emplace_back(polygon::create_random(
                    Vector2(xs, ys), 4));

                //polygons.emplace_back(polygon::create_circle(Vector2(xs, ys), 5));
                
                auto& polygon = polygons.back();

                polygon.get_physical_object().rotate(M_PI / 2);

                /*if(draw_circle)
                    polygons.emplace_back(polygon::create_circle(Vector2(xs, ys), 5));
                else
                    polygons.emplace_back(polygon::create_random(Vector2(xs, ys), 
                        polygon_vertex_count));

     
                const auto& shape = polygon.get_shape();

                std::uniform_int_distribution<unsigned> rnd(0, shape.getPointCount() - 1);
                const auto random_point = polygon.get_shape().getPoint(rnd(rng));

                polygons.back().get_physical_object().accelerate(
                    as_world_coordinates(random_point), { 0, 15.0f });*/

                increase_polygon = true;

            } break;

            case sf::Event::MouseButtonReleased:
            {
                increase_polygon = false;

                polygons.at(polygons.size() - 1)
                    .get_physical_object()
                    .set_type(object_type::dynamic);

                std::cout << polygons.size() << std::endl;

            } break;

            case sf::Event::KeyPressed:
            {
                if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape) ||
                    sf::Keyboard::isKeyPressed(sf::Keyboard::Q))
                {
                    window.close();
                }

				if (sf::Keyboard::isKeyPressed(sf::Keyboard::G))
				{
					game_loop_type = static_cast<GameLoopType>(static_cast<int>(game_loop_type) + 1);
					if (game_loop_type == GameLoopType::Unkown)
						game_loop_type = GameLoopType::Fixed;


				}

                /*
                if (sf::Keyboard::isKeyPressed(sf::Keyboard::I))
                {
                polygon_vertex_count = ((polygon_vertex_count + 1) % 7) + 3;
                }

                if (sf::Keyboard::isKeyPressed(sf::Keyboard::D))
                {
                polygon_vertex_count = ((polygon_vertex_count - 1) % 7) + 3;
                }

                if (sf::Keyboard::isKeyPressed(sf::Keyboard::C))
                {
                polygon_vertex_count = circle;
                }*/

            } break;

            default: break;
            }
        }
    };

	auto fixed_game_loop = [&]()
	{
		auto elapsed_ms = duration_cast<milliseconds>(clock::now() - last_time);

		if (elapsed_ms < interval)
		{
			//auto interval_ms = duration_cast<milliseconds>(interval);
			//auto wait = interval_ms - elapsed_ms;
			std::this_thread::sleep_for(0ms);
			return;
		}

		last_time = clock::now();

		process_events();

		while (elapsed_ms >= interval)
		{
			if (increase_polygon)
			{
				polygons.back().scale(1 + interval.count() * 2);
			}

			measure("Update", update_time_smooth, [&]
			{
				update(polygons, interval.count());
			});

			elapsed_ms = duration_cast<milliseconds>(elapsed_ms - interval);
		}

		Console::instance().set_param("Update", update_time_smooth.get());

		measure("Draw", draw_time_smooth, [&]
		{
			render(window, polygons);
		});
	};

	auto variable_game_loop = [&]()
	{
		auto elapsed_s = duration_cast<duration<double>>(clock::now() - last_time);
		last_time = clock::now();

		process_events();

		if (increase_polygon)
			polygons.back().scale(1 + elapsed_s.count() * 2);


		measure("Update", update_time_smooth, [&]
		{
			update(polygons, elapsed_s.count());
		});

		measure("Draw", draw_time_smooth, [&]
		{
			render(window, polygons);
		});
	};

	while (window.isOpen())
	{
		switch (game_loop_type)
		{
		case GameLoopType::Fixed:
			Console::instance().set_param("GameLoop", "Fixed");
			fixed_game_loop();
			break;
		case GameLoopType::Variable:
			Console::instance().set_param("GameLoop", "Variable");
			variable_game_loop();
			break;
		case GameLoopType::Unkown:
		default:
			break;
		}
	}


    return 0;
}