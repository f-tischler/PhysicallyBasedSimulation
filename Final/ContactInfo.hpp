#pragma once

#include "Polygon.h"
#include "Vec2.h"

struct ContactInfo
{
	polygon* a;
	polygon* b;
	std::vector<Vector2> contact_points;
};