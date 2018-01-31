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

const sf::Color clear_color({ 10, 10, 20 });

enum class game_loop
{
	fixed = 0,
	variable,
	unkown
};

/*----------------------------------------------------------------*/

void render(sf::RenderWindow& window, const std::vector<polygon>& polygons,
            const sf::VertexArray& custom_polygon)
{
    window.clear(clear_color);
    
    for (auto& polygon : polygons)
    {
        polygon.draw(window);
    }

    for (auto& polygon : polygons)
    {
        polygon.draw_debug(window);
    }

	console::instance().print(window);

    if(custom_polygon.getVertexCount() != 0)
    {
        window.draw(custom_polygon);
        
        for(auto i = 0; i < custom_polygon.getVertexCount(); ++i)
        {
            sf::CircleShape corner(4);
            corner.setOrigin(corner.getRadius(), corner.getRadius());
            corner.setPosition(custom_polygon[i].position);
            corner.setFillColor(sf::Color::Transparent);
            corner.setOutlineColor(sf::Color::Yellow);
            corner.setOutlineThickness(2);

            window.draw(corner);
        }
    }


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
    auto increase_polygon = false;
    auto create_custom_polygon = false;

    auto polygon_vertex_count = 4;
    auto scroll_vertex_count = 0;

    auto custom_polygon = sf::VertexArray(sf::Lines);

	console::instance().init();

    constexpr auto MAX_VERTICES = 10;
    constexpr auto width = 1280;
    constexpr auto height = 800;

    sf::RenderWindow window(sf::VideoMode(width, height), "2D Collision detection");

    std::vector<polygon> polygons;
	{
		Vector2d position(400, 700);
		Vector2d scale(710, 40);

		polygons.emplace_back(polygon::create_rectangle(position, scale));
		polygons.back().get_physical_object().rotate(-5 * M_PI / 180);
		polygons.back().scale(1);
	}

	{
		Vector2d position(900, 650);
		Vector2d scale(700, 40);

		polygons.emplace_back(polygon::create_rectangle(position, scale));
		polygons.back().get_physical_object().rotate(30 * M_PI / 180);
		polygons.back().scale(1);
	}

	{
		Vector2d position(600, 400);
		polygons.emplace_back(polygon::create_random(position, 6));
		polygons.back().scale(8);
	}

	{
		Vector2d position(200, 420);
		polygons.emplace_back(polygon::create_random(position, 6));
		polygons.back().scale(8);
	}

	{
		Vector2d position(900, 300);
		polygons.emplace_back(polygon::create_circle(position, 60));
	}

    const auto static_scene_size = polygons.size();

    using namespace std::chrono;
    using clock = high_resolution_clock;

	auto game_loop_type = game_loop::variable;

	smoother<double, 10> draw_time_smooth;
	smoother<double, 10> update_time_smooth;

    auto last_time = clock::now();

    std::default_random_engine rng;

    auto process_events = [&, xs = 0, ys = 0, debug = false] () mutable
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

            case sf::Event::MouseWheelScrolled:
            {
                scroll_vertex_count++;
                if(scroll_vertex_count >= 10)
                {
                    scroll_vertex_count = 0;
                    polygon_vertex_count = ( (polygon_vertex_count - 2) %
                                             (MAX_VERTICES - 3) ) + 3;
                }
            } break;

            case sf::Event::MouseButtonPressed:
            {
                if(event.mouseButton.button == sf::Mouse::Left)
                {
                    custom_polygon.append(sf::Vertex(sf::Vector2f(xs,ys),
                                                     sf::Color::Yellow));
                    if(create_custom_polygon)
                        custom_polygon.append(sf::Vertex(sf::Vector2f(xs,ys),
                                                         sf::Color::Yellow));
                    create_custom_polygon = true;

                } else if(event.mouseButton.button == sf::Mouse::Right)
                {
                    if(create_custom_polygon)
                    {
                        // cancel
                        custom_polygon.clear();
                        create_custom_polygon = false;
                    }
                    else
                    {
                        polygons.emplace_back(polygon::create_random(
                            Vector2d(xs, ys), polygon_vertex_count));

                        auto& polygon = polygons.back();

                        polygon.get_physical_object().rotate(M_PI / 2);

                        increase_polygon = true;
                    }
                }

            } break;

            case sf::Event::MouseButtonReleased:
            {
                if(increase_polygon)
                {
                    increase_polygon = false;

                    polygons.at(polygons.size() - 1)
                        .get_physical_object()
                        .set_type(object_type::dynamic);
                }

            } break;

            case sf::Event::KeyPressed:
            {
                if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape) ||
                    sf::Keyboard::isKeyPressed(sf::Keyboard::Q))
                {
                    window.close();
                }

                if (sf::Keyboard::isKeyPressed(sf::Keyboard::R))
                {
                    while (polygons.size() > static_scene_size)
                        polygons.pop_back();
                }

				if (sf::Keyboard::isKeyPressed(sf::Keyboard::G))
				{
					game_loop_type = static_cast<game_loop>(static_cast<int>(game_loop_type) + 1);
					if (game_loop_type == game_loop::unkown)
						game_loop_type = game_loop::fixed;
				}

                if (sf::Keyboard::isKeyPressed(sf::Keyboard::D))
                {
                    debug = !debug;

                    for (auto& polygon : polygons)
                        polygon.enable_debug_info(debug);
                }

                if(sf::Keyboard::isKeyPressed(sf::Keyboard::Return))
                {
                    if(create_custom_polygon)
                    {
                        create_custom_polygon = false;

                        polygons.emplace_back(
                            polygon::create_custom(custom_polygon));
                        
                        custom_polygon.clear();

                        polygons.back().get_physical_object
                                ().rotate(0);

                        polygons.back()
                                .get_physical_object()
                                .set_type(object_type::dynamic);

                        polygons.back().enable_debug_info(debug);
                    }
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
			render(window, polygons,custom_polygon);
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
			render(window, polygons,custom_polygon);
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

        console::instance().set_param("Vertex Count", polygon_vertex_count);

	}

    return 0;
}