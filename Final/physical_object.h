#ifndef PHYSICAL_OBJECT_H
#define PHYSICAL_OBJECT_H

#include "Eigen/Core"
#include "Eigen/Geometry"

using Eigen::Vector2d;
using Eigen::Matrix2d;
using Rotation2D = Eigen::Rotation2D<double>;

class physical_object
{
public:
    explicit physical_object(
        const Vector2d position,
        const std::vector<std::tuple<Vector2d, double>>& points);

    void update(const double dt);
    void accelerate(Vector2d point, Vector2d acceleration);
    void accelerate(Vector2d force);

    Vector2d position() const { return position_; }
    Rotation2D rotation() const { return rotation_; }
    Vector2d center_of_mass() const { return center_of_mass_; }

private:
    double mass_;

    Vector2d center_of_mass_; 
    Vector2d position_;
    Rotation2D rotation_;

    Vector2d velocity_;
    Vector2d force_;

    double torque_;
    double angular_momentum_;
    double angular_velocity_;
    double inverse_inertia_;
};

#endif // PHYSICAL_OBJECT_H
