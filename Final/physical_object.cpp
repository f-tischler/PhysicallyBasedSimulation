#include "physical_object.h"
#include <numeric>

constexpr auto world_scale = 50; // px = 1m

physical_object::physical_object(const Vector2d position,
    const std::vector<std::tuple<Vector2d, double>>& points)
    : position_(position), rotation_(0), velocity_(0, 0), force_(0, 0), torque_(0), angular_momentum_(0)
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
        [cof = center_of_mass_](auto current_sum, auto p)
    {
        const Vector2d offset = std::get<0>(p) - cof;
        return current_sum + std::get<1>(p) * offset.squaredNorm();
    });

    inverse_inertia_ = 1 / inertia;

    // initial angular velocity
    angular_velocity_ = inverse_inertia_ * angular_momentum_;
}

void physical_object::update(const double dt)
{
    center_of_mass_ += velocity_ * dt;
    position_ += velocity_ * dt;

    velocity_ += force_ / mass_ * dt * world_scale;
    rotation_ = Rotation2D(rotation_.angle() + angular_velocity_ * dt * world_scale);

    angular_momentum_ += torque_ * dt;
    angular_velocity_ = inverse_inertia_ * angular_momentum_;

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