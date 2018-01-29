#include "physical_object.h"
#include <numeric>

physical_object::physical_object(const Vector2d position,
    const std::vector<std::tuple<Vector2d, double>>& points)
    : scale_(1), radius_(0), position_(position), rotation_(0), 
    velocity_(0, 0), force_(0, 0), torque_(0), angular_momentum_(0), angular_velocity_(0), points_(points.size())
{
    // total mass
    initial_mass_ = std::accumulate(points.begin(), points.end(), 0.0, 
        [](auto current_sum, auto p)
    {
        return current_sum + std::get<1>(p);
    });

    mass_ = initial_mass_;

    // initial position of center of mass
    center_of_mass_ = std::accumulate(points.begin(), points.end(), Vector2d(0, 0),
        [](auto current_sum, auto p)
    {
        return current_sum + std::get<0>(p) * std::get<1>(p);
    }) / mass_;

    // get offsets
    for(const auto& point : points)
    {
        initial_offsets_.push_back(
            std::get<0>(point) - center_of_mass_);
    }

    update_points();
}

void physical_object::update(const double dt)
{
    switch (type_)
    {
    case object_type::fixed: return;
    case object_type::dynamic: accelerate(gravity);
    default: break;
    }

    position_ += velocity_ * dt;

    velocity_ += force_ / mass_ * dt;
    rotation_ = Rotation2D(rotation_.angle() + angular_velocity_ * dt);

    angular_momentum_ += torque_ * dt;
    angular_velocity_ = inverse_inertia_ * angular_momentum_;

    update_points();

    force_ = { 0.0, 0.0 };
    torque_ = 0.0;
}

void physical_object::accelerate(const Vector2d point, const Vector2d acceleration)
{
    accelerate(acceleration);

    const auto offset = point - center_of_mass_;
    const auto force = acceleration * mass_;

    torque_ += cross2(offset, force);
}

void physical_object::accelerate(const Vector2d acceleration)
{
    force_ += acceleration * mass_;
}

void physical_object::add_force(Vector2d force)
{
    force_ += force;
}

void physical_object::update_points()
{
    auto inertia = 0.0;
    for (auto i = 0u; i < points_.size(); ++i)
    {
        // transform offset
        const auto offset = rotation_.toRotationMatrix() * initial_offsets_[i] * scale_;

        // calculate norm of angular velocity part (v = r * omega)
        //const auto velocity_norm = offset.norm() * angular_velocity_;
        //const auto normalized_offset = offset.normalized();

        points_[i] =
        {
            offset,
            velocity_ + cross2(angular_velocity_, offset)
        };

        // save maximum offset
        radius_ = std::max(radius_, offset.norm());

        // update inertia
        inertia += initial_mass_ / points().size() * offset.squaredNorm();
    }

    inverse_inertia_ = 1.0 / inertia;
}
