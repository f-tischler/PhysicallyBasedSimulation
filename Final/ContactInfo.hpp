#pragma once

#include "physical_object.h"

using line_t = std::tuple<point_t, point_t>;

struct contact_info
{
    contact_info(
        physical_object& point_owner, 
        const point_t& point,
        physical_object& line_owner,
        const line_t& line)
        : point_owner(point_owner),
          point(point),
          line_owner(line_owner),
          line(line)
    {
    }

	physical_object& point_owner;
    point_t point;

    physical_object& line_owner;
    line_t line;
};