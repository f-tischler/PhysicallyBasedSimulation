//
// Created by ivan on 16/01/18.
//

#ifndef PHYSICALLYBASEDSIMULATION_POLYGON_H
#define PHYSICALLYBASEDSIMULATION_POLYGON_H

#include <vector>
#include <SFML/Graphics.hpp>

#include "Vec2.h"
#include "physical_object.h"

const Vector2 gravity = Vector2(0, 9.81);
//const Vector2 gravity = Vector2(0, 01);

class polygon 
{
public:
    polygon(const Vector2& center, std::vector<Vector2> points);

    static polygon create_rectangle(const Vector2 pos, const Vector2 scale);
    static polygon create_line(const Vector2 start, const Vector2 end);
    static polygon create_circle(const Vector2 center, const double radius);
    static polygon create_random(const Vector2 center, const size_t vertex_count);

    void update(const double dt);
    void draw(sf::RenderWindow& window) const;

    const sf::Shape& get_shape() const { return shape_; }
    physical_object& get_physical_object() { return physical_object_; }

    void increase(double dt);

    void enable();

    friend std::ostream& operator<<(std::ostream& os, const polygon& p);

private:
    sf::ConvexShape shape_;
    sf::CircleShape cof_shape_;
   
    physical_object physical_object_;

    bool enabled_;

    void update_shapes();
};

inline sf::Vector2f to_sf_vector(Vector2d v)
{
    return 
    { 
        static_cast<float>(v.x()), 
        static_cast<float>(v.y())
    };
}

inline Vector2d to_eigen_vector(const Vector2& v)
{
    return { v.x(), v.y() };
}

inline Vector2d to_eigen_vector(const sf::Vector2f& v)
{
    return { v.x, v.y };
}

#endif //PHYSICALLYBASEDSIMULATION_POLYGON_H
