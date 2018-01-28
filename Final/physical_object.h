#ifndef PHYSICAL_OBJECT_H
#define PHYSICAL_OBJECT_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

using Eigen::Vector2d;
using Eigen::Matrix2d;
using Rotation2D = Eigen::Rotation2D<double>;

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Vector2d)

static const Vector2d gravity = { 0, -9.81 };

enum class object_type
{
    fixed,
    dynamic,
    kinematic
};

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

    void set_type(const object_type type) { type_ = type; }
    object_type get_type() const { return type_; }

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

    std::vector<Vector2d> offsets_;
    std::vector<Vector2d> velocities_;
    object_type type_ = object_type::fixed;
};

#endif // PHYSICAL_OBJECT_H
