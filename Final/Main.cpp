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


void intersects(const polygon& a, const polygon& b, std::vector<Vector2>& contact_points)
{
	const auto get_points = [](const physical_object& object)
	{
		using line_t = std::tuple<Vector2d, Vector2d>;

		std::vector<line_t> lines;

        const auto position = 
            object.position() +
            object.center_of_mass_local();

        const auto& points = object.get_points();

        for (size_t i = 0; i < points.size(); i++)
        {
            auto end_index = i == points.size() - 1
                ? 0
                : i + 1;

            auto start = position + std::get<0>(points[i]);
			auto end = position + std::get<0>(points[end_index]);

			lines.emplace_back(start, end);
		}

		return lines;
	};

    const auto& object_a = a.get_physical_object();
    const auto& object_b = b.get_physical_object();

    if ((object_a.center_of_mass_global() -
        object_b.center_of_mass_global()).norm() >
        object_a.bounding_radius() +
        object_b.bounding_radius())
        return;

	auto lines_a = get_points(object_a);
	auto lines_b = get_points(object_b);

	for (auto& line_a : lines_a)
	{
		for (auto& line_b : lines_b)
		{
			auto x1 = std::get<0>(line_a).x();
			auto y1 = std::get<0>(line_a).y();

			auto x2 = std::get<1>(line_a).x();
			auto y2 = std::get<1>(line_a).y();

			auto x3 = std::get<0>(line_b).x();
			auto y3 = std::get<0>(line_b).y();

			auto x4 = std::get<1>(line_b).x();
			auto y4 = std::get<1>(line_b).y();

			double ix, iy;
			if(!lineSegmentIntersection(x1, y1, x2, y2, x3, y3, x4, y4, ix, iy))
                continue;

			contact_points.emplace_back(ix, iy);
		}
	}
}

std::vector<ContactInfo> collision_detection(std::vector<polygon>& polygons)
{
	std::vector<ContactInfo> constacts;
	for (auto& polygon_a : polygons)
	{
		polygon_a.clear_contacts();

		for (auto& polygon_b : polygons)
		{
			if (&polygon_a == &polygon_b)
				continue;

			if (polygon_a.get_physical_object().get_type() == object_type::fixed &&
                polygon_b.get_physical_object().get_type() == object_type::fixed)
				continue;

			ContactInfo info;
			intersects(polygon_a, polygon_b, info.contact_points);
			if (!info.contact_points.empty())
			{
				polygon_a.add_contacts(info.contact_points);
				polygon_b.add_contacts(info.contact_points);

				info.a = &polygon_a;
				info.b = &polygon_b;
				constacts.push_back(std::move(info));
			}
		}
	}

	return constacts;
}

void collision_resolution(const std::vector<ContactInfo>& contacts)
{
	
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
		smoother.add(1.0 / elapsed_s.count());
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