//
// Created by ivan on 16/01/18.
//

#ifndef PHYSICALLYBASEDSIMULATION_POLYGON_H
#define PHYSICALLYBASEDSIMULATION_POLYGON_H

#include "physical_object.h"

#include <SFML/Graphics.hpp>
#include <vector>
#include "ContactInfo.hpp"

constexpr auto screen_scale = 20; // px = 1m

class polygon
{
public:
    polygon(const Vector2d& center, std::vector<Vector2d> points);

    static polygon create_rectangle(const Vector2d& pos, const Vector2d& scale);
    static polygon create_circle(const Vector2d& center, const double radius);
    static polygon create_random(const Vector2d& center, const size_t vertex_count);
    static polygon create_custom(sf::VertexArray custom_polygon);

    void update(const double dt);
    void draw(sf::RenderWindow& window) const;
    void draw_debug(sf::RenderWindow& window) const;

    const sf::Shape& get_shape() const { return shape_; }

    physical_object& get_physical_object() { return physical_object_; }
    const physical_object& get_physical_object() const { return physical_object_; }

    void scale(double dt);
    void update_color(const sf::Color& color);
    void set_center(const Vector2d new_center);

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

    void enable_debug_info(const bool enable) { debug_output_ = enable; }
    void toggle_debug_info() { debug_output_ = !debug_output_; }

    friend std::ostream& operator<<(std::ostream& os, const polygon& p);

private:
    sf::ConvexShape shape_;
    sf::CircleShape cof_shape_;
   
    physical_object physical_object_;
    std::vector<contact_info> contacts_;

    bool debug_output_ = false;

    sf::Color color_;

    void update_shapes();

    void set_color(const sf::Color& color);
};

inline sf::Vector2f as_screen_coordinates(const Vector2d& v)
{
    return 
    { 
        static_cast<float>(v.x()) * screen_scale,
        -static_cast<float>(v.y() * screen_scale)
    };
}

inline Vector2d as_world_coordinates(const sf::Vector2f& v)
{
    return { v.x / screen_scale, -v.y / screen_scale};
}

inline Vector2d as_world_coordinates(const Vector2d& v)
{
    return { v.x() / screen_scale, -v.y() / screen_scale };
}

inline sf::Vector2f to_sf(const Vector2d& v)
{
    return
    {
        static_cast<float>(v.x()),
        static_cast<float>(v.y())
    };
}


#endif //PHYSICALLYBASEDSIMULATION_POLYGON_H
