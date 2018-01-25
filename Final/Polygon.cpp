//
// Created by ivan on 16/01/18.
//

#include <vector>
#include <random>

#define _USE_MATH_DEFINES
#include <math.h>

#include "Polygon.h"
#include <SFML/System/Vector2.hpp>

using namespace std;
std::default_random_engine rng;

polygon::polygon(const Vector2& center, std::vector<Vector2> points)
    : velocity_({0,0}), enabled_(false)
{


    shape_.setPointCount(points.size());

    for (auto i = 0u; i < points.size(); i++)
    {
        auto point = points[i];

        shape_.setPoint(i, sf::Vector2f(
            static_cast<float>(point.x()),
            static_cast<float>(point.y())));
    }

    shape_.setPosition(
        static_cast<float>(center.x()),
        static_cast<float>(center.y()));
}

void polygon::update(std::vector<polygon>& polygons, const double dt)
{
    if (!enabled_) return;

    shape_.move(
        static_cast<float>(velocity_.x() * dt),
        static_cast<float>(velocity_.y() * dt));

    velocity_ += gravity * dt;

    for(const auto & poly : polygons) {
        if(this == &poly)
            continue;
        if (shape_.getPointCount() == 100 && poly.shape_.getPointCount() == 100)
        {
            sf::Vector2f distance = { shape_.getPosition() - poly.shape_.getPosition() };
            if( (this->shape_.getScale() + poly.shape_.getScale()).x > Vector2(distance.x,distance.y).length() )
                shape_.move(
                        static_cast<float>(- velocity_.x() * dt),
                        static_cast<float>(- velocity_.y() * dt));
        }

    }
}


void polygon::increase(const double factor)
{
    shape_.scale(
        static_cast<float>(factor),
        static_cast<float>(factor));
}

void polygon::enable()
{
    enabled_ = true;
}

std::ostream& operator<<(std::ostream& os, const polygon& p)
{
    os << "Polygon:";

    const auto& shape = p.get_shape();

    for (auto i = 0u; i < shape.getPointCount(); ++i)
    {
        const auto point = shape.getPoint(i);

        os<< " " << point.x << ":" << point.y;
    }

    return os;
}

polygon polygon::create_rectangle(const Vector2 pos, const Vector2 scale)
{
    const vector<Vector2> points =
    {
        Vector2(0,0),
        Vector2(0, scale.y()),
        scale,
        Vector2(scale.x(), 0)
    };

    return { pos, points };
}

polygon polygon::create_line(const Vector2 start, const Vector2 end)
{
    const vector<Vector2> points =
    {
        Vector2(0,0),
        Vector2(0.0001, 0.0001),
        end,
        Vector2(end.x() - 0.0001, end.y() - 0.0001)
    };

    return { (end - start) / 2, points };
}

polygon polygon::create_circle(const Vector2 center, const double radius)
{
    constexpr auto vertex_count = 100;

    const auto angle = 360.0 / vertex_count;

    vector<Vector2> points;
    for (auto i = 0; i < vertex_count; ++i) 
    {
        points.emplace_back(
            cos(i * angle * M_PI / 180.0), 
            sin(i * angle * M_PI / 180.0));
    }

    polygon p(center, points);

    p.increase(radius);

    return p;
}

polygon polygon::create_random(const Vector2 center, const size_t vertex_count)
{
    const auto angle = 360.0 / vertex_count;

    const std::uniform_real_distribution rnd_distr_distance(- angle / vertex_count, angle / vertex_count);

    vector<Vector2> points;
    for (auto i = 0u; i < vertex_count; ++i)
    {
        const auto final_angle = angle + rnd_distr_distance(rng);
        points.emplace_back(
            std::cos(i * final_angle * M_PI / 180.0),
            std::sin(i * final_angle * M_PI / 180.0));
    }

    return { center, points };
}

