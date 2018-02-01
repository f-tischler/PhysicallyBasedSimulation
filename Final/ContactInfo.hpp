#pragma once

#include "physical_object.h"


class contact_info
{
public:
    contact_info(physical_object& point_owner,
        const point_t& point,
        physical_object& line_owner,
        const line_t& line,
        const double penetration_depth,
        const Vector2d& point_of_intersection)
        : point_owner_(point_owner),
        point_(point),
        line_owner_(line_owner),
        line_(line),
        penetration_depth_(penetration_depth),
        point_of_intersection_(point_of_intersection)
    {
    }

    physical_object& line_owner() const { return line_owner_; }
    physical_object& point_owner() const { return point_owner_; }

    const point_t& line_start_point() const { return line_owner_.points()[std::get<0>(line_)]; }
    const point_t& line_end_point() const { return line_owner_.points()[std::get<1>(line_)]; }

    Vector2d line_start_offset() const { return std::get<0>(line_start_point()); }
    Vector2d line_end_offset() const { return std::get<0>(line_end_point()); }
    Vector2d line_offset() const { return (line_start_offset() + line_end_offset()) / 2.0; }

    const point_t& contact_point() const { return point_; }
    const Vector2d& contact_point_offset() const { return std::get<0>(point_); }
    const Vector2d& contact_point_velocity() const { return std::get<1>(point_); }

    Vector2d relative_velocity() const
    {
        const Vector2d line_velocity =
            (std::get<1>(line_start_point())
           + std::get<1>(line_end_point())) / 2.0;

        return contact_point_velocity() - line_velocity;
    }
    
    Vector2d normal() const
    {
        return ::normal(line_start_offset(), line_end_offset());
    }

    double penetration_depth() const { return penetration_depth_; }

private:
	physical_object& point_owner_;
    point_t point_;

    physical_object& line_owner_;
    line_t line_;

    double penetration_depth_;
    Vector2d point_of_intersection_;
};