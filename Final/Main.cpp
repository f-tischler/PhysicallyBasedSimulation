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
#include <iostream>
#include <vector>
#include <SFML/Graphics.hpp>

/* Local includes */
#include "Polygon.h"
#include "Console.hpp"
#include "smoother.h"

#include "collision_detection.h"
#include "collision_resolution.h"

enum class game_loop
{
	fixed = 0,
	variable,
	unkown
};

/*----------------------------------------------------------------*/

void render(sf::RenderWindow& window, const std::vector<polygon>& polygons)
{
    window.clear();
    
    for (auto& polygon : polygons)
    {
        polygon.draw(window);
    }

	console::instance().print(window);

    window.display();
}

void update(std::vector<polygon>& polygons, const double dt)
{
    auto contacts = measure("Update: Collision Detection", [&]
    {
        return collision_detection(polygons);
    });
	
    measure("Update: Collision Resolution", [&]
    {
	    collision_resolution(contacts);
    });

    measure("Update: Integration", [&]
    {
        for (auto& polygon : polygons)
        {
            polygon.update(dt);
        }
    });

    measure("Update: Position Correction", [&]
    {
        correct_positions(contacts);
    });
}

int main()
{
	console::instance().init();

    constexpr auto width = 1280;
    constexpr auto height = 800;

    sf::RenderWindow window(sf::VideoMode(width, height), "2D Collision detection");

    std::vector<polygon> polygons;

	polygons.emplace_back(polygon::create_rectangle(Vector2d(400, 700), Vector2d(710, 40)));
    polygons.back().get_physical_object().rotate(-5 * M_PI / 180);
    
    polygons.emplace_back(polygon::create_rectangle(Vector2d(900, 650), Vector2d(700, 40)));
    polygons.back().get_physical_object().rotate(30 * M_PI / 180);

    polygons.emplace_back(polygon::create_rectangle(Vector2d(50, 400), Vector2d(700, 40)));
    polygons.back().get_physical_object().rotate(-70 * M_PI / 180);

    polygons.emplace_back(polygon::create_random(Vector2d(600, 400), 6));
    polygons.back().scale(8);

    polygons.emplace_back(polygon::create_random(Vector2d(200, 420), 4));
    polygons.back().scale(5);

    polygons.emplace_back(polygon::create_circle(Vector2d(900, 300), 60));

    using namespace std::chrono;
    using clock = high_resolution_clock;

	auto game_loop_type = game_loop::variable;

	smoother<double, 10> draw_time_smooth;
	smoother<double, 10> update_time_smooth;

    auto last_time = clock::now();

    std::default_random_engine rng;

    auto increase_polygon = false;

    auto process_events = [&, xs = 0, ys = 0] () mutable
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
                    Vector2d(xs, ys), 4));

                //polygons.emplace_back(polygon::create_circle(Vector2(xs, ys), 5));
  
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
					game_loop_type = static_cast<game_loop>(static_cast<int>(game_loop_type) + 1);
					if (game_loop_type == game_loop::unkown)
						game_loop_type = game_loop::fixed;
				}

                if (sf::Keyboard::isKeyPressed(sf::Keyboard::D))
                {
                    for (auto& polygon : polygons)
                        polygon.toggle_debug_info();
                }

            } break;

            default: break;
            }
        }
    };

	const auto fixed_game_loop = [&, interval = duration<double>(16ms)]()
	{
		auto elapsed_ms = duration_cast<milliseconds>(clock::now() - last_time);

		if (elapsed_ms < interval)
		{
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

		console::instance().set_param("Update", update_time_smooth.get());

		measure("Draw", draw_time_smooth, [&]
		{
			render(window, polygons);
		});
	};

	const auto variable_game_loop = [&]()
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
		case game_loop::fixed:
			console::instance().set_param("GameLoop", "Fixed");
			fixed_game_loop();
			break;
		case game_loop::variable:
			console::instance().set_param("GameLoop", "Variable");
			variable_game_loop();
			break;
		case game_loop::unkown:
		default:
			break;
		}
	}

    return 0;
}