#ifndef PHYSICAL_OBJECT_H
#define PHYSICAL_OBJECT_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#define _USE_MATH_DEFINES
#include <math.h>

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
    void add_linear_velocity(const Vector2d v) { velocity_ += v; update_points(); }
    void add_angular_velocity(const double v) { angular_velocity_ += v; update_points(); }

    Vector2d position() const { return position_; }
    Rotation2D rotation() const { return rotation_; }
    Vector2d linear_velocity() const { return velocity_; }
    Vector2d center_of_mass_local() const { return center_of_mass_; }
    Vector2d center_of_mass_global() const { return position_ + center_of_mass_; }
    double inverse_inertia() const { return inverse_inertia_; }
    double bounding_radius() const { return radius_; }
    const std::vector<point_t>& points() const { return points_; }

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

    void set_scale(const double scale)
    {
        scale_ = scale; 
        mass_ = initial_mass_ * scale;
        
        // updates offsets, inertia 
        // and bounding radius
        update_points();
    }
    double get_scale() const { return scale_; }

    void set_type(const object_type type) { type_ = type; }
    object_type get_type() const { return type_; }

    void move(const Vector2d& v)
    {
        position_ += v;
    }

    void rotate(const double a)
    {
        rotation_ = Rotation2D(rotation_.angle() + a);
        update_points();
    }

private:
    double initial_mass_;
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

inline double cross2(const Vector2d a, const Vector2d b)
{
    return a.x() * b.y() - b.x() * a.y();
}

inline Vector2d cross2(const Vector2d v, const double s)
{
    return { s * v.y(), -s * v.x() };
}

inline Vector2d cross2(const double s, const Vector2d v)
{
    return { -s * v.y(), s * v.x() };
}

#endif // PHYSICAL_OBJECT_H
