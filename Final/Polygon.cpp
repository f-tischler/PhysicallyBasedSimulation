//
// Created by ivan on 16/01/18.
//

#include <vector>
#include <random>

#define _USE_MATH_DEFINES
#include <math.h>

#include "Polygon.h"
#include <SFML/System/Vector2.hpp>

std::default_random_engine rng;

std::vector<std::tuple<Vector2d, double>> create_mass_points(
    const double mass_per_point,
    const std::vector<Vector2>& points)
{
    std::vector<std::tuple<Vector2d, double>> mass_points;
    for(const auto& p : points)
    {
        mass_points.push_back({ to_eigen_vector(p), mass_per_point });
    }

    return mass_points;
}

polygon::polygon(const Vector2& center, std::vector<Vector2> points)
    : physical_object_(to_eigen_vector(center), create_mass_points(0.5, points)), enabled_(false)
{
    shape_.setPointCount(points.size());

    for (auto i = 0u; i < points.size(); i++)
    {
        auto point = points[i];

        shape_.setPoint(i, sf::Vector2f(
            static_cast<float>(point.x()),
            static_cast<float>(point.y())));
    }

    shape_.setOutlineThickness(-0.05);
    shape_.setOrigin(
    {
        static_cast<float>(physical_object_.center_of_mass().x()),
        -static_cast<float>(physical_object_.center_of_mass().y())
    });

    cof_shape_.setRadius(0.05f);
    cof_shape_.setOrigin({ cof_shape_.getRadius(), cof_shape_.getRadius() });
    cof_shape_.setFillColor(sf::Color::Yellow);
    
    update_shapes();
}

void polygon::update(const double dt)
{
    if (!enabled_) return;

    physical_object_.accelerate({ gravity.x(), gravity.y() });
    physical_object_.update(dt);

    update_shapes();
}

void polygon::update_shapes()
{
    shape_.setRotation(static_cast<float>(physical_object_.rotation().angle() * 180 / M_PI));
    shape_.setPosition(to_sf_vector(physical_object_.position()));

    cof_shape_.setPosition(to_sf_vector(physical_object_.position()));
}

void polygon::draw(sf::RenderWindow& window) const
{
    window.draw(shape_);
    window.draw(cof_shape_);
}


void polygon::scale(const double factor)
{
    shape_.scale(
        static_cast<float>(factor),
        static_cast<float>(factor));

    cof_shape_.scale(
        static_cast<float>(factor),
        static_cast<float>(factor));

}

void polygon::set_color(const sf::Color& color)
{
    const sf::Color dark = {
        static_cast<sf::Uint8>(color.r / 2),
        static_cast<sf::Uint8>(color.g / 2),
        static_cast<sf::Uint8>(color.b / 2),
    };

    shape_.setFillColor(dark);
    shape_.setOutlineColor(color);
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
    const auto aspect = scale.x() / scale.y();
    const auto half_height = 0.5;
    const auto half_width = half_height * aspect;

    const std::vector<Vector2> points =
    {
        Vector2(-half_width, -half_height),
        Vector2(-half_width,  half_height),
        Vector2( half_width,  half_height),
        Vector2( half_width, -half_height)
    };

    polygon p(pos, points);

    p.scale(scale.x() / half_width / 2);
    p.set_color(sf::Color::Blue);

    return p;
}

polygon polygon::create_line(const Vector2 start, const Vector2 end)
{
    const std::vector<Vector2> points =
    {
        Vector2(0,0),
        Vector2(0.0001, 0.0001),
        end,
        Vector2(end.x() - 0.0001, end.y() - 0.0001)
    };

    polygon p((end - start) / 2, points);

    p.set_color(sf::Color::White);

    return p;
}

polygon polygon::create_circle(const Vector2 center, const double radius)
{
    constexpr auto vertex_count = 100;

    const auto angle = 360.0 / vertex_count;

    std::vector<Vector2> points;
    for (auto i = 0; i < vertex_count; ++i) 
    {
        points.emplace_back(
            cos(i * angle * M_PI / 180.0), 
            sin(i * angle * M_PI / 180.0));
    }

    polygon p(center, points);

    p.scale(radius);
    p.set_color(sf::Color::Red);

    return p;
}

polygon polygon::create_random(const Vector2 center, const size_t vertex_count)
{
    const auto angle = 2 * M_PI / vertex_count;

    const std::uniform_real_distribution<double> rnd(
        -angle / vertex_count, angle / vertex_count);

    std::vector<Vector2> points;
    for (auto i = 0u; i < vertex_count; ++i)
    {
        const auto final_angle = i * angle + rnd(rng);
        points.emplace_back(
            std::cos(final_angle),
            std::sin(final_angle));
    }

    polygon p(center, points);

    p.scale(5);
    p.set_color(sf::Color::Green);

    return p;
}
