//
// Created by ivan on 16/01/18.
//

#include <vector>
#include <random>

#include "Polygon.h"
#include <SFML/System/Vector2.hpp>

std::default_random_engine rng;

std::vector<std::tuple<Vector2d, double>> create_mass_points(
    const double density,
    const std::vector<Vector2d>& points)
{
    constexpr auto depth = 1.0;

    // Calculate value of shoelace formula
    auto area = 0.0;
    auto j = points.size() - 1;

    for (auto i = 0u; i < points.size(); i++)
    {
        const auto point_i = points[i];
        const auto point_j = points[j];

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

polygon::polygon(const Vector2d& center, std::vector<Vector2d> points)
    : physical_object_(as_world_coordinates(center), create_mass_points(0.05, points))
{
    shape_.setPointCount(points.size());

    for (auto i = 0u; i < points.size(); i++)
    {
        shape_.setPoint(i, to_sf(points[i]));
    }

    const auto cof = as_screen_coordinates(physical_object_.center_of_mass_local());

    shape_.setOutlineThickness(-0.05f);
    shape_.setOrigin(cof);

    cof_shape_.setFillColor(sf::Color::Yellow);
    cof_shape_.setRadius(0.05f);
    cof_shape_.setOrigin(
    { 
        cof_shape_.getRadius(), 
        cof_shape_.getRadius() 
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
    shape_.setRotation(-static_cast<float>(physical_object_.rotation().angle() * 180 / M_PI));
    shape_.setPosition(as_screen_coordinates(physical_object_.position()));

    cof_shape_.setPosition(as_screen_coordinates(physical_object_.center_of_mass_global()));
}

void polygon::draw_debug(sf::RenderWindow& window) const
{
    sf::CircleShape circle(2);

    circle.setOutlineColor(sf::Color{ 255, 0, 255 });
    circle.setOutlineThickness(1);
    circle.setOrigin(circle.getRadius(), circle.getRadius());

    sf::Vertex normal_line[2];

    for (const auto& contact : contacts_)
    {
        const auto center = physical_object_.center_of_mass_global();
        const auto point = center + std::get<0>(contact.point);

        circle.setPosition(as_screen_coordinates(point));
        window.draw(circle);

        const auto line_start = std::get<0>(std::get<0>(contact.line));
        const auto line_end = std::get<0>(std::get<1>(contact.line));

        const auto n = normal(line_start, line_end);

        normal_line[0] = sf::Vertex(as_screen_coordinates(point), sf::Color{ 255, 127, 0 });
        normal_line[1] = sf::Vertex(as_screen_coordinates(point + n), sf::Color{ 255, 127, 0 });

        window.draw(normal_line, 2, sf::Lines);
    }
}

void polygon::draw(sf::RenderWindow& window) const
{
    window.draw(shape_);
    window.draw(cof_shape_);

    if (!debug_output_) return;

    draw_debug(window);
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

polygon polygon::create_rectangle(const Vector2d pos, const Vector2d size)
{
    const auto aspect = size.x() / size.y();
    const auto half_height = 0.5;
    const auto half_width = half_height * aspect;

    std::vector<Vector2d> points; 
    points.emplace_back(-half_width, -half_height);
    points.emplace_back(-half_width, half_height);
    points.emplace_back(half_width, half_height);
    points.emplace_back(half_width, -half_height);

    polygon p(pos, points);

    p.scale(size.x() / half_width / 2);
    p.set_color(sf::Color::Blue);

    return p;
}

polygon polygon::create_circle(const Vector2d center, const double radius)
{
    constexpr auto vertex_count = 20;

    const auto angle = 360.0 / vertex_count;

    std::vector<Vector2d> points;
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

polygon polygon::create_random(const Vector2d center, const size_t vertex_count)
{
    const auto angle = 2 * M_PI / vertex_count;

    std::uniform_real_distribution<double> rnd(
        -angle / vertex_count, angle / vertex_count);

    std::vector<Vector2d> points;
    for (auto i = 0u; i < vertex_count; ++i)
    {
        const auto final_angle = i * angle + rnd(rng);
        points.emplace_back(
            std::cos(final_angle),
            std::sin(final_angle));
    }

    polygon p(center, points);

    p.scale(15);
    p.set_color(sf::Color::Green);

    return p;
}
