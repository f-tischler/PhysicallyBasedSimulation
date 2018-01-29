#ifndef PHYSICAL_OBJECT_H
#define PHYSICAL_OBJECT_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

using Eigen::Vector2d;
using Eigen::Matrix2d;
using Rotation2D = Eigen::Rotation2D<double>;

// offset, velocity
using point_t = std::tuple<Vector2d, Vector2d>;

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Vector2d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(point_t)

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
    void accelerate(Vector2d acceleration);
    void add_force(Vector2d force);

    Vector2d position() const { return position_; }
    Rotation2D rotation() const { return rotation_; }
    Vector2d linear_velocity() const { return velocity_; }
    void add_linear_velocity(const Vector2d v) { velocity_ += v; }

    double mass() const 
    { 
        return type_ == object_type::fixed
        ? 0
        : mass_; 
    }

    double inverse_mass() const
    {
        return type_ == object_type::fixed
            ? 0
            : 1.0 / mass_;
    }

    Vector2d center_of_mass_local() const { return center_of_mass_; }
    Vector2d center_of_mass_global() const { return position_ + center_of_mass_; }

    double bounding_radius() const { return radius_; }

    void set_scale(const double scale)
    {
        scale_ = scale; 
        update_points();
    }

    double get_scale() const { return scale_; }

    void set_type(const object_type type) { type_ = type; }
    object_type get_type() const { return type_; }

    const std::vector<point_t>& points() const { return points_; }
    void move(const Vector2d& v) { position_ += v; }


private:
    double mass_;
    double scale_;
    double radius_;

    Vector2d center_of_mass_; 
    Vector2d position_;
    Rotation2D rotation_;

    Vector2d velocity_;
    Vector2d force_;

    double torque_;
    double angular_momentum_;
    double angular_velocity_;
    double inverse_inertia_;

    std::vector<Vector2d> initial_offsets_;
    std::vector<point_t> points_;

    object_type type_ = object_type::fixed;

    void update_points();
};

#endif // PHYSICAL_OBJECT_H
