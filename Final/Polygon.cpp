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
    const double density,
    const std::vector<Vector2>& points)
{
    constexpr auto depth = 1.0;

    // Calculate value of shoelace formula
    auto area = 0.0;
    auto j = points.size() - 1;

    for (auto i = 0u; i < points.size(); i++)
    {
        auto point_i = points[i];
        auto point_j = points[j];

        area += (point_j.x() + point_i.x()) * (point_j.y() - point_i.y());
        j = i; // j is previous vertex to i
    }

    area = abs(area / 2.0);

    const auto total_mass = area * depth * density;
    const auto mass_per_point = total_mass / points.size();

    std::vector<std::tuple<Vector2d, double>> mass_points;
    for(const auto& p : points)
    {
        mass_points.push_back({ as_world_coordinates(p), mass_per_point });
    }

    return mass_points;
}

polygon::polygon(const Vector2& center, std::vector<Vector2> points)
    : physical_object_(as_world_coordinates(center), create_mass_points(0.5, points))
{
    shape_.setPointCount(points.size());

	auto accum = Vector2{ 0, 0 };
    for (auto i = 0u; i < points.size(); i++)
    {
        auto point = points[i];
		accum += point;

        shape_.setPoint(i, sf::Vector2f(
            static_cast<float>(point.x()),
            static_cast<float>(point.y())));
    }

    const auto cof = as_screen_coordinates(physical_object_.center_of_mass_local());

    shape_.setOutlineThickness(-0.05);
    shape_.setOrigin(cof);

    cof_shape_.setFillColor(sf::Color::Yellow);
    cof_shape_.setRadius(0.05f);
    cof_shape_.setOrigin(
    { 
        cof.x + cof_shape_.getRadius(), 
        cof.y + cof_shape_.getRadius() 
    });

    update_shapes();
}

void polygon::update(const double dt)
{
    physical_object_.update(dt);

    update_shapes();
}

void polygon::update_shapes()
{
    shape_.setRotation(static_cast<float>(physical_object_.rotation().angle() * 180 / M_PI));
    shape_.setPosition(as_screen_coordinates(physical_object_.position()));

    cof_shape_.setPosition(as_screen_coordinates(physical_object_.position()));
}

void polygon::draw(sf::RenderWindow& window) const
{
    window.draw(shape_);
    window.draw(cof_shape_);

    // draw contact
    for (const auto& contact : contacts_)
    {
        const auto center = physical_object_.center_of_mass_global();

        const auto point = center + std::get<0>(contact.point);
        
        sf::CircleShape circle(3);
        circle.setPosition(as_screen_coordinates(point));
        circle.setFillColor(sf::Color{255, 0, 255});
        window.draw(circle);
        
        const auto direction = point - center;

        sf::Vertex line[] =
        {
            sf::Vertex(as_screen_coordinates(center), sf::Color{255, 0, 255}),
            sf::Vertex(as_screen_coordinates(center + direction), sf::Color{255, 0, 255})
        };

        window.draw(line, 2, sf::Lines);

        const auto position_line_owner = contact.line_owner.center_of_mass_global();
        const auto position_point_owner = contact.line_owner.center_of_mass_global();

        const auto line_start = std::get<0>(std::get<0>(contact.line));
        const auto line_end = std::get<0>(std::get<1>(contact.line));

        const auto normal = Vector2d(
            -(line_end - line_start).y(),
            (line_end - line_start).x()
        ).normalized();

        sf::Vertex normal_line[] =
        {
            sf::Vertex(as_screen_coordinates(point), sf::Color{ 255, 127, 0 }),
            sf::Vertex(as_screen_coordinates(point + normal * 5), sf::Color{ 255, 127, 0 })
        };

        window.draw(normal_line, 2, sf::Lines);
    }
}


void polygon::scale(const double factor)
{
    shape_.scale(
        static_cast<float>(factor),
        static_cast<float>(factor));

    cof_shape_.scale(
        static_cast<float>(factor),
        static_cast<float>(factor));

    physical_object_.set_scale(shape_.getScale().x);
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

polygon polygon::create_rectangle(const Vector2 pos, const Vector2 size)
{
    const auto aspect = size.x() / size.y();
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

    p.scale(size.x() / half_width / 2);
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
