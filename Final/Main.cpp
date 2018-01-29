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
        line_t line;
        auto intersections = 0;
        auto distance = std::numeric_limits<double>::max();

        for (const auto& line_b : lines_b)
        {
            const auto x1 = position_a.x() + std::get<0>(point_a).x();
            const auto y1 = position_a.y() + std::get<0>(point_a).y();

            const auto x2 = position_a.x() + std::get<0>(point_a).x() * 100000.0;
            const auto y2 = position_a.y() + std::get<0>(point_a).y() * 100000.0;

            const auto x3 = position_b.x() + std::get<0>(std::get<0>(line_b)).x();
            const auto y3 = position_b.y() + std::get<0>(std::get<0>(line_b)).y();

            const auto x4 = position_b.x() + std::get<0>(std::get<1>(line_b)).x();
            const auto y4 = position_b.y() + std::get<0>(std::get<1>(line_b)).y();

            double ix, iy;
            if (!lineSegmentIntersection(x1, y1, x2, y2, x3, y3, x4, y4, ix, iy))
                continue;

            ++intersections;

            const auto current_distance = 
                (Vector2d(ix, iy) - Vector2d(x1, y2)).norm();

            if (current_distance >= distance)
                continue;

            distance = current_distance;
            line = line_b;
        }

        if (intersections % 2 == 0) continue;

        contacts.emplace_back(object_a, point_a, object_b, line);
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

        const auto normal = Vector2d(
            -(line_end - line_start).y(),
            (line_end - line_start).x()
        ).normalized();

        auto& a = contact.line_owner;
        auto& b = contact.point_owner;

        constexpr auto e = 0.99;

        const auto relative_linear_velocity = 
            (a.linear_velocity() - b.linear_velocity()).dot(normal);

        //if(relative_linear_velocity < 0)
        //    continue;

        const auto j = -(1 + e) * relative_linear_velocity /
            (normal.dot(normal) * (a.inverse_mass() + b.inverse_mass()));

        const auto impulse = j * normal;

        const auto total_mass = a.mass() + b.mass();

        assert(total_mass > 0);

        a.add_linear_velocity(impulse * a.inverse_mass() * a.mass() / total_mass);
        b.add_linear_velocity(-impulse * b.inverse_mass() * b.mass() / total_mass);
    }
}

void update(std::vector<polygon>& polygons, const double dt)
{
	const auto contacts = collision_detection(polygons);
	
	if (!contacts.empty())
		collision_resolution(contacts);

	for (auto& polygon : polygons)
	{
		polygon.update(dt);
	}
}

int main()
{
    auto increase_polygon = false;
    auto polygon_vertex_count = 3;

    std::vector<polygon> polygons;

    double xs = 0;
    double ys = 0;

    constexpr auto width = 600;
    constexpr auto height = 600;

	Console::instance().init();

    sf::RenderWindow window(sf::VideoMode(width, height), "2D Collision detection");

	polygons.emplace_back(polygon::create_rectangle(Vector2(200, 500), Vector2(300, 50)));
    polygons.emplace_back(polygon::create_rectangle(Vector2(200, 200), Vector2(300, 50)));

    using namespace std::chrono;
    using clock = high_resolution_clock;

	const auto fps_60 = duration<double>(16ms);
	const auto fps_30 = duration<double>(32ms);
    const auto interval = fps_60;


	auto game_loop_type = GameLoopType::Fixed;

	smoother<double, 100> draw_time_smooth;
	smoother<double, 100> update_time_smooth;

    auto last_time = clock::now();

	auto measure = [&](auto name, auto& smoother, auto func) {

		auto start_frame = clock::now();

		func();

		auto elapsed_s = duration<double>{ clock::now() - start_frame };
		smoother.add(elapsed_s.count());
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
                /*polygons.emplace_back(polygon::create_random(
                Vector2(xs, ys), polygon_vertex_count));*/

                polygons.emplace_back(polygon::create_circle(Vector2(xs, ys), 5));
                
                //auto& polygon = polygons.back();

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