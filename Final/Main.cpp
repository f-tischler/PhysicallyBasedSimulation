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
#include <memory>

/* Local includes */
#include "Polygon.h"
#include "globals.h"
#include "chrono_timer.h"


/*----------------------------------------------------------------*/

bool add_polygon = false;
bool increase_polygon = false;
bool random_polygon = false;
int polygon = 3;
std::vector<std::shared_ptr<Polygon> > polygons (0);

double xs = 0;
double ys = 0;

int width  = 600;
int height  = 600;

long long dt = 0;

int main()
{
    sf::RenderWindow window(sf::VideoMode(width, height), "2D Collision detection");
    ChronoTimer t("Unnamed");

    {
        Polygon p = Polygon::Circle(Vector2(200,300),30);
        std::shared_ptr < Polygon > sp = std::make_shared<Polygon>(p);
        polygons.push_back(sp);
        Polygon p1 = Polygon::Square(Vector2(200,500),Vector2(-300,50));
        std::shared_ptr < Polygon > sp1 = std::make_shared<Polygon>(p1);
        polygons.push_back(sp1);
    }
    while (window.isOpen())
    {
        dt = t.get_milliseconds();

        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed ||
                    sf::Keyboard::isKeyPressed(sf::Keyboard::Escape) ||
                    sf::Keyboard::isKeyPressed(sf::Keyboard::Q) )
                window.close();
            if(event.type == sf::Event::MouseMoved) { // <- this is how you check to see if an event
                xs = sf::Mouse::getPosition().x - window.getPosition().x - 10;
                ys = sf::Mouse::getPosition().y - window.getPosition().y - 35;
            }
            if(event.type == sf::Event::MouseButtonPressed) { // <- this is how you check to see if an event
                Polygon p=Polygon(Vector2(xs,ys),polygon);
                std::shared_ptr<Polygon> sp = std::make_shared<Polygon>(p);
                polygons.push_back(sp);
                increase_polygon = true;
            }
            if(event.type == sf::Event::MouseButtonReleased) { // <- this is how you check to see if an event
                increase_polygon = false;
                polygons.at(polygons.size()-1)->set_ready();

            }
            if(sf::Keyboard::isKeyPressed(sf::Keyboard::I)) { // <- this is how you check to see if an event
                polygon = ((polygon + 1) % 7) + 3;
            }
            if(sf::Keyboard::isKeyPressed(sf::Keyboard::D)) { // <- this is how you check to see if an event
                polygon = ((polygon - 1) % 7) + 3;
            }
            if(sf::Keyboard::isKeyPressed(sf::Keyboard::C)) { // <- this is how you check to see if an event
                polygon = circle;
            }
        }
        if(increase_polygon)
            polygons.at(polygons.size()-1)->increase(.01);

        std::cout<<polygons.size()<<std::endl;
        window.clear();
        for(const auto& polygon : polygons) {
            window.draw(polygon->draw());
            polygon->update(t.get_milliseconds() - dt);
        }
        window.display();
    }

    return 0;
}