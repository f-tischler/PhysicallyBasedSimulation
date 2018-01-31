#include "physical_object.h"
#include <numeric>

physical_object::physical_object(const Vector2d& position,
    const std::vector<std::tuple<Vector2d, double>>& points)
    : scale_(1), radius_(0), position_(position), rotation_(0), 
    velocity_(0, 0), force_(0, 0), torque_(0), angular_velocity_(0), points_(points.size())
{
    // total mass
    initial_mass_ = std::accumulate(points.begin(), points.end(), 0.0, 
        [](auto current_sum, auto p)
    {
        return current_sum + std::get<1>(p);
    });

    mass_ = initial_mass_;

    // initial position of center of mass
	auto tmp = Vector2d(0, 0);
    center_of_mass_ = std::accumulate(points.begin(), points.end(), tmp,
        [](const auto& current_sum, auto p)
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

void physical_object::perform_symplectic_euler_step(const double dt)
{
    const auto a = force_ / mass_;
    const auto a_m = inverse_inertia_ * torque_;

    velocity_ += a * dt;
    angular_velocity_ += a_m * dt;

    position_ += velocity_ * dt;
    rotation_ = Rotation2D(rotation_.angle() + angular_velocity_ * dt);
}

void physical_object::update(const double dt)
{
    switch (type_)
    {
    case object_type::fixed: return;
    case object_type::dynamic: accelerate(gravity);
    default: break;
    }

    perform_symplectic_euler_step(dt);

    update_points();

    assert(velocity_.squaredNorm() < 100000000);
    assert(std::abs(angular_velocity_) < 100000000);
    assert(position_.squaredNorm() < 100000000);
    assert(std::abs(rotation_.angle()) < 100000000);

    force_ = { 0.0, 0.0 };
    torque_ = 0.0;
}

void physical_object::accelerate(const Vector2d& point, const Vector2d& acceleration)
{
    accelerate(acceleration);

    const auto offset = point - center_of_mass_;
    const auto force = acceleration * mass_;

    torque_ += cross2(offset, force);
}

void physical_object::accelerate(const Vector2d& acceleration)
{
    force_ += acceleration * mass_;
}

void physical_object::update_points()
{
    auto inertia = 0.0;
    for (auto i = 0u; i < points_.size(); ++i)
    {
        // transform offset
		auto rot = rotation_.toRotationMatrix();
        const auto offset = rot * initial_offsets_[i] * scale_;

        points_[i] =
        {
            offset,
            velocity_ + cross2(angular_velocity_, offset)
        };

        // save maximum offset
        radius_ = std::max(radius_, offset.norm());

        // update inertia
        inertia += mass_ / points().size() * offset.squaredNorm();
    }

    inverse_inertia_ = 1.0 / inertia;
}


void physical_object::position(const Vector2d& new_center) {
    this->position_ = new_center;
}
