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
#include <thread>


/*----------------------------------------------------------------*/

void render(sf::RenderWindow& window, const std::vector<polygon>& polygons)
{
    window.clear();

    for (auto& polygon : polygons)
    {
        window.draw(polygon.get_shape());
    }

    window.display();
}

void update(std::vector<polygon>& polygons, const double dt)
{
    for (auto& polygon : polygons)
    {
        polygon.update(polygons,dt);
    }
}

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

    sf::RenderWindow window(sf::VideoMode(width, height), "2D Collision detection");

    polygons.emplace_back(polygon::create_circle(Vector2(200, 300), 30));
    polygons.emplace_back(polygon::create_rectangle(Vector2(200, 500), Vector2(300, 50)));

    using namespace std::chrono;
    using clock = high_resolution_clock;

    const auto interval = duration<double>(10ms);
    
    auto last_time = clock::now();

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
                    polygons.emplace_back(polygon::create_random(
                Vector2(xs, ys), polygon_vertex_count));


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

    while (window.isOpen())
    {
        auto elapsed_ms = duration_cast<milliseconds>(clock::now() - last_time);

        if (elapsed_ms < interval)
        {
            std::this_thread::sleep_for(0ms);
            continue;
        }

        last_time = clock::now();

        process_events();
        
        while(elapsed_ms >= interval)
        {
            if (increase_polygon)
            {
                polygons.back().increase(1 + interval.count() * 2);
            }

            update(polygons, interval.count());

            elapsed_ms = duration_cast<milliseconds>(elapsed_ms - interval);
        }

        render(window, polygons);
    }

    return 0;
}