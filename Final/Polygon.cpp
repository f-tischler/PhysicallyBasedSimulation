//
// Created by ivan on 16/01/18.
//

#include <math.h>
#include <vector>
#include <random>
#include <stdlib.h>

#include <iostream>
#include "Polygon.h"
#include "globals.h"

#define PI 3.14159265


using namespace std;

Polygon::Polygon(Vector2 pos) : center(pos.x(),pos.y()),
                                               vertices(4),
                                               scale(1),
                                               velocity({0,0}),
                                               ready(false) {
    this->points = vector<Vector2>(vertices);
}

Polygon::Polygon(Vector2 pos, int vertices) : center(pos.x(),pos.y()),
                                               vertices((vertices==10)?100:vertices),
                                               scale(1),
                                               velocity({0,0}),
                                               ready(false) {
    if (vertices != circle) {
        this->points = vector<Vector2>(vertices);
        const auto angle = 360.0 / (double)vertices;

        std::uniform_real_distribution rnd_distr_distance(-angle/2, angle/2);

        for (int i = 0; i < vertices; ++i) {
            const Vector2 direction = {cos((double)i * (angle + rnd_distr_distance(rng)) * PI / 180),
                                       sin((double)i * (angle + rnd_distr_distance(rng)) * PI / 180)};

            this->points[i] = direction;
        }
    } else {
        this->points = vector<Vector2>(vertices);

        const auto angle = 360.0 / (double)vertices;

        for (int i = 0; i < vertices; ++i) {
            const Vector2 direction = {cos((double)i * angle * PI / 180), sin((double)i * angle * PI / 180)};

            this->points[i] = direction;
        }
    }
}

void Polygon::update(double dt) {
    dt /= 50;
    if(ready) {
        this->velocity += gravity * dt;
        auto old_center = this->center;
        this->center += this->velocity * dt;
        /////////////////////////////////
        //////COLLISION DETECTION////////
        /////////////////////////////////
    }
}

sf::ConvexShape Polygon::draw() const {
    // create an empty shape
    sf::ConvexShape convex;

    convex.setPointCount(points.size());

    for(int i=0;i<points.size();i++) {
        Vector2 point=this->center - (points[i] * this->scale);
        convex.setPoint(i, sf::Vector2f(point.x(), point.y()));
    }
    return convex;
}

void Polygon::increase(double dt) {
    this->scale+=dt;
}

void Polygon::set_ready() {
    this->ready=true;
}

std::ostream& operator<<(std::ostream& os, const Polygon& p)
{
    os << "Polygon:";
    for (auto & point : p.points) {
        Vector2 real_pos =
                p.center + (p.scale * point);
        os<< " " << real_pos.x() << ":" << real_pos.y();
    }
    return os;
}

Polygon Polygon::Square(Vector2 pos, Vector2 scale){
    Polygon retval = Polygon(pos);
    retval.points[0] = Vector2(0,0);
    retval.points[1] = Vector2(0,scale.y());
    retval.points[2] = scale;
    retval.points[3] = Vector2(scale.x(),0);
    return retval;
}
Polygon Polygon::Line(Vector2 start, Vector2 end){
    Polygon retval = Polygon(start);

    retval.points[0] = Vector2(0,0);
    retval.points[1] = Vector2(0.0001,0.0001);
    retval.points[2] = end;
    retval.points[3] = Vector2(end.x()-0.0001,end.y()-0.0001);
    return retval;
}

Polygon Polygon::Circle(Vector2 center, double radius){
    Polygon retval = Polygon(center,circle);
    retval.increase(radius);
    return retval;
}