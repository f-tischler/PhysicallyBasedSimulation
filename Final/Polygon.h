//
// Created by ivan on 16/01/18.
//

#ifndef PHYSICALLYBASEDSIMULATION_POLYGON_H
#define PHYSICALLYBASEDSIMULATION_POLYGON_H


#include "Vec2.h"

#include <SFML/Graphics.hpp>

#include <vector>


constexpr auto world_scale = 50; // px = 1m
// const Vector2 gravity = Vector2(0, 9.81) * world_scale;
const Vector2 gravity = Vector2(0, 0.1) * world_scale;

class polygon 
{
public:
    polygon(const Vector2& center, std::vector<Vector2> points);

    static polygon create_rectangle(const Vector2 pos, const Vector2 scale);
    static polygon create_line(const Vector2 start, const Vector2 end);
    static polygon create_circle(const Vector2 center, const double radius);
    static polygon create_random(const Vector2 center, const size_t vertex_count);

    void update(double dt);

    const sf::ConvexShape& get_shape() const { return shape_; }

    void increase(double dt);

    void enable();

	bool get_enabled() const { return enabled_; }

	void add_contacts(const std::vector<Vector2>& contacts)
	{
		for (auto& contact : contacts)
			contacts_.push_back(contact);
	}

	void clear_contacts() 
	{
		contacts_.clear();
	}

	const std::vector<Vector2>& get_contacts() const
	{
		return contacts_;
	}

	const Vector2& get_center_local() const
	{
		return center_;
	}

    friend std::ostream& operator<<(std::ostream& os, const polygon& p);

private:
    sf::ConvexShape shape_;
    Vector2 velocity_;
	Vector2 center_;
    bool enabled_;
	double area_;
	double mass_;
	std::vector<Vector2> contacts_;
};

#endif //PHYSICALLYBASEDSIMULATION_POLYGON_H
