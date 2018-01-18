//
// Created by ivan on 16/01/18.
//

#ifndef PHYSICALLYBASEDSIMULATION_POLYGON_H
#define PHYSICALLYBASEDSIMULATION_POLYGON_H

#include <vector>
#include <random>
#include <SFML/Graphics.hpp>

#include "Vec2.h"

static const Vector2 gravity = Vector2(0, 9.81);

class Polygon {
    const int vertices;
    double scale;
    Vector2 center;
    Vector2 velocity;
    std::vector<Vector2> points;
    bool ready;

    Polygon(Vector2 pos);

public:

    Polygon(Vector2 pos, int vertices);

    static Polygon Square(Vector2 pos, Vector2 scale);
    static Polygon Line(Vector2 start, Vector2 end);
    static Polygon Circle(Vector2 center, double radius);

    void update(double dt);
    sf::ConvexShape draw() const;
    void increase(double dt);
    void set_ready();
    friend std::ostream& operator<<(std::ostream& os, const Polygon& p);

};

#endif //PHYSICALLYBASEDSIMULATION_POLYGON_H
