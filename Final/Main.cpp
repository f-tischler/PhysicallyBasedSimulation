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
#include <iostream>
#include <vector>
#include <SFML/Graphics.hpp>

/* Local includes */
#include "Polygon.h"
#include "Console.hpp"
#include <thread>
#include <numeric>
#include <random>


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
        window.draw(polygon.get_shape());
    }

	Console::instance().print(window);

    window.display();
}

void update(std::vector<polygon>& polygons, const double dt)
{
    for (auto& polygon : polygons)
    {
        polygon.update(dt);
    }
}

#include<deque>

template<typename T, unsigned int MAX>
class Smoother
{
public:

	void add(T v)
	{
		data.push_back(v);
		if (data.size() > MAX)
			data.pop_front();
	}

	T get()
	{
		return std::accumulate(data.begin(), data.end(), 0.0) / data.size();
	}

private:
	std::deque<T> data;
};

int main()
{
    auto increase_polygon = false;
    auto draw_circle = false;
    auto polygon_vertex_count = 3;

    std::vector<polygon> polygons;

    double xs = 0;
    double ys = 0;

    constexpr auto width = 600;
    constexpr auto height = 600;

	Console::instance().init();

    sf::RenderWindow window(sf::VideoMode(width, height), "2D Collision detection");

    polygons.emplace_back(polygon::create_circle(Vector2(200, 300), 30));
    polygons.emplace_back(polygon::create_rectangle(Vector2(200, 500), Vector2(300, 50)));

    using namespace std::chrono;
    using clock = high_resolution_clock;

	const auto fps_60 = duration<double>(16ms);
	const auto fps_30 = duration<double>(32ms);
    const auto interval = fps_60;


	auto game_loop_type = GameLoopType::Fixed;

	Smoother<double, 100> draw_time_smooth;
	Smoother<double, 100> update_time_smooth;

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
                if(draw_circle)
                    polygons.emplace_back(polygon::create_circle(Vector2(xs, ys), 5));
                else
                    polygons.emplace_back(polygon::create_random(Vector2(xs, ys), 
                        polygon_vertex_count));

                auto& polygon = polygons.back();
                const auto& shape = polygon.get_shape();

                std::uniform_int_distribution<unsigned> rnd(0, shape.getPointCount() - 1);
                const auto random_point = polygon.get_shape().getPoint(rnd(rng));

                polygons.back().get_physical_object().accelerate(
                    to_eigen_vector(random_point), { 0, -150.0f });

                increase_polygon = true;

            } break;

            case sf::Event::MouseButtonReleased:
            {
                increase_polygon = false;

                polygons.at(polygons.size() - 1).enable();

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
					game_loop_type = (GameLoopType)(((int)game_loop_type) + 1);
					if (game_loop_type == GameLoopType::Unkown)
						game_loop_type = GameLoopType::Fixed;


				}

                if(event.key.code > sf::Keyboard::Num2 && event.key.code < sf::Keyboard::Escape)
                {
                    polygon_vertex_count = event.key.code - sf::Keyboard::Num0;
                }


                if (sf::Keyboard::isKeyPressed(sf::Keyboard::C))
                {
                draw_circle = true;
                }

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
			auto interval_ms = duration_cast<milliseconds>(interval);
			auto wait = interval_ms - elapsed_ms;
			std::this_thread::sleep_for(0ms);
			return;
		}

		last_time = clock::now();

		process_events();

		while (elapsed_ms >= interval)
		{
			if (increase_polygon)
			{
				polygons.back().increase(1 + interval.count() * 2);
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
			polygons.back().increase(1 + elapsed_s.count() * 2);


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