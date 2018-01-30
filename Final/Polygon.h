//
// Created by ivan on 16/01/18.
//

#ifndef PHYSICALLYBASEDSIMULATION_POLYGON_H
#define PHYSICALLYBASEDSIMULATION_POLYGON_H

#include "Vec2.h"
#include "physical_object.h"

#include <SFML/Graphics.hpp>
#include <vector>
#include "ContactInfo.hpp"

constexpr auto screen_scale = 20; // px = 1m

class polygon
{
public:
    polygon(const Vector2& center, std::vector<Vector2> points);

    static polygon create_rectangle(const Vector2 pos, const Vector2 scale);
    static polygon create_line(const Vector2 start, const Vector2 end);
    static polygon create_circle(const Vector2 center, const double radius);
    static polygon create_random(const Vector2 center, const size_t vertex_count);
    static polygon create_custom(sf::VertexArray custom_polygon);

    void update(const double dt);
    void draw(sf::RenderWindow& window) const;

    const sf::Shape& get_shape() const { return shape_; }

    physical_object& get_physical_object() { return physical_object_; }
    const physical_object& get_physical_object() const { return physical_object_; }

    void scale(double dt);

    void add_contacts(const std::vector<contact_info>& contacts)
    {
        for (auto& contact : contacts)
        {
            if (&contact.point_owner != &physical_object_)
                continue;

            contacts_.push_back(contact);
        }
    }

    void clear_contacts()
    {
        contacts_.clear();
    }

    const Vector2& get_center_local() const
    {
        return
        {
            physical_object_.center_of_mass_local().x(),
            physical_object_.center_of_mass_local().y()
        };
    }

    void enable_debug_info(const bool enable) { debug_output_ = enable; }
    void toggle_debug_info() { debug_output_ = !debug_output_; }

    friend std::ostream& operator<<(std::ostream& os, const polygon& p);

private:
    sf::ConvexShape shape_;
    sf::CircleShape cof_shape_;
   
    physical_object physical_object_;
    std::vector<contact_info> contacts_;

    bool debug_output_ = false;

    void update_shapes();
    void draw_debug(sf::RenderWindow& window) const;
    void set_color(const sf::Color& color);
};

inline sf::Vector2f as_screen_coordinates(Vector2d v)
{
    return 
    { 
        static_cast<float>(v.x()) * screen_scale,
        -static_cast<float>(v.y() * screen_scale)
    };
}

inline Vector2d as_world_coordinates(const Vector2& v)
{
    return { v.x() / screen_scale, -v.y() / screen_scale};
}

inline Vector2d as_world_coordinates(const sf::Vector2f& v)
{
    return { v.x / screen_scale, -v.y / screen_scale };
}

#endif //PHYSICALLYBASEDSIMULATION_POLYGON_H
