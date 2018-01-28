#include "physical_object.h"
#include <numeric>

physical_object::physical_object(const Vector2d position,
    const std::vector<std::tuple<Vector2d, double>>& points)
    : position_(position), rotation_(0), velocity_(0, 0), force_(0, 0), 
    torque_(0), angular_momentum_(0), points_(points.size()), scale_(1), radius_(0)
{
    // total mass
    mass_ = std::accumulate(points.begin(), points.end(), 0.0, 
        [](auto current_sum, auto p)
    {
        return current_sum + std::get<1>(p);
    });

    // initial position of center of mass
    center_of_mass_ = std::accumulate(points.begin(), points.end(), Vector2d(0, 0),
        [](auto current_sum, auto p)
    {
        return current_sum + std::get<0>(p) * std::get<1>(p);
    }) / mass_;

    // inertia
    const auto inertia = std::accumulate(points.begin(), points.end(), 0.0,
        [cof = center_of_mass_, this](auto current_sum, auto p)
    {
        const Vector2d offset = std::get<0>(p) - cof;

        // save offset
        initial_offsets_.push_back(offset);

        return current_sum + std::get<1>(p) * offset.squaredNorm();
    });

    inverse_inertia_ = 1 / inertia;

    // initial angular velocity
    angular_velocity_ = inverse_inertia_ * angular_momentum_;

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

    torque_ += offset.x() * force.y() - force.x() * offset.y();
}

void physical_object::accelerate(const Vector2d acceleration)
{
    force_ += acceleration * mass_;
}

void physical_object::update_points()
{
    for (auto i = 0u; i < points_.size(); ++i)
    {
        // transform offset
        const auto offset = rotation_.toRotationMatrix() * initial_offsets_[i] * scale_;

        // calculate norm of angular velocity part (v = r * omega)
        const auto velocity_norm = offset.norm() * angular_velocity_;
        const auto normalized_offset = offset.normalized();

        points_[i] =
        {
            offset,
            velocity_ + Vector2d
            {
                velocity_norm * normalized_offset.y(),
                velocity_norm * normalized_offset.x(),
            }
        };

        // save maximum offset
        radius_ = std::max(radius_, offset.norm());
    }
}
