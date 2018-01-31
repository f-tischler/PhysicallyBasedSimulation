#ifndef COLLISION_RESOLUTION_H
#define COLLISION_RESOLUTION_H

#include "ContactInfo.hpp"

inline void apply_friction(
    physical_object& a, 
    physical_object& b, 
    const Vector2d& offset_a,
    const Vector2d& offset_b,
    const Vector2d& normal,
    const Vector2d& rel_v,
    const double rel_v_n,
    const double j,
    const int num_contacts)
{
    const auto tangent = (rel_v - normal * rel_v_n)
        .normalized();

    const auto ra_n = cross2(offset_a, tangent);
    const auto rb_n = cross2(offset_b, tangent);

    const auto t_a = a.inverse_mass() + a.inverse_inertia() * ra_n * ra_n;
    const auto t_b = b.inverse_mass() + b.inverse_inertia() * rb_n * rb_n;
    const auto denom = (t_a + t_b) * num_contacts;

    assert(std::abs(denom) > 0);

    const auto jt = -rel_v.dot(tangent) / denom;

    if (std::abs(jt) < 0.000001) return;

    const auto static_friction = 0.61;
    const auto dynamic_friction = 0.47;

    const auto tangent_impulse = std::abs(jt) < j * static_friction
        ? (tangent * jt).eval()
        : (tangent * -j * dynamic_friction).eval();

    a.apply_impulse(-tangent_impulse, offset_a);
    b.apply_impulse(tangent_impulse, offset_b);
}

inline double calc_impulse_norm(
    const physical_object& a,
    const physical_object& b,
    const Vector2d& offset_a,
    const Vector2d& offset_b,
    const Vector2d& normal,
    const double rel_v_n,
    const double restitution,
    const int num_contacts)
{
    const auto ra_n = cross2(offset_a, normal);
    const auto rb_n = cross2(offset_b, normal);

    const auto t_a = a.inverse_mass() + a.inverse_inertia() * ra_n * ra_n;
    const auto t_b = b.inverse_mass() + b.inverse_inertia() * rb_n * rb_n;
    const auto denom = (t_a + t_b) * num_contacts;

    assert(std::abs(denom) > 0);

    return -(1 + restitution) * rel_v_n / denom;
}

inline void apply_impulse(
    physical_object& a, 
    physical_object& b, 
    const Vector2d& offset_a, 
    const Vector2d& offset_b,
    const Vector2d n, 
    const double j)
{
    const auto normal_impulse = j * n;

    a.apply_impulse(-normal_impulse, offset_a);
    b.apply_impulse( normal_impulse, offset_b);
}

inline void collision_resolution(const std::vector<contact_info>& contacts, const double dt)
{
    for (const auto& contact : contacts)
    {
        auto& a = contact.line_owner();
        auto& b = contact.point_owner();

        const auto n = contact.normal();

        const auto contact_points = std::count_if(contacts.begin(), contacts.end(),
            [&b, &a](const contact_info& c)
        {
            return  &c.point_owner() == &b && &c.line_owner() == &a ||
                    &c.point_owner() == &a && &c.line_owner() == &b;
        });

        assert(contact_points > 0);

        const auto rel_v = contact.relative_velocity();
        const auto rel_v_n = rel_v.dot(n);

        if (rel_v_n > 0)
            continue;

        // if relative velocity 
        const auto e = rel_v.squaredNorm() < (gravity * dt).squaredNorm() + 0.01
            ? 0
            : 0.3;

        const auto j = calc_impulse_norm(a, b, 
            contact.line_offset(), 
            contact.contact_point_offset(), 
            n, 
            rel_v_n, 
            e,
            contact_points);

        assert(std::abs(j) < 100000);

        apply_impulse(a, b, 
            contact.line_offset(), 
            contact.contact_point_offset(), 
            n, j);

        apply_friction(a, b, 
            contact.line_offset(), 
            contact.contact_point_offset(), 
            n, rel_v, rel_v_n, j, 
            contact_points);
    }
}

inline void correct_positions(const std::vector<contact_info>& contacts)
{
    for (const auto& contact : contacts)
    {
        auto& a = contact.line_owner();
        auto& b = contact.point_owner();

        const auto n = contact.normal();

        const auto percent = 0.5; // usually 20% to 80%
        const auto slop = 0.01; // usually 0.01 to 0.1

        const auto contact_points = std::count_if(contacts.begin(), contacts.end(),
            [&b, &a](const contact_info& c)
        {
            return  &c.point_owner() == &b && &c.line_owner() == &a ||
                    &c.point_owner() == &a && &c.line_owner() == &b;
        });

        const Vector2d correction = std::max(contact.penetration_depth() - slop, 0.0)
            / (a.inverse_mass() + b.inverse_mass()) * percent * n ;

        a.move(-a.inverse_mass() * correction / contact_points);
        b.move(b.inverse_mass() * correction / contact_points);
    }
}
#endif // COLLISION_RESOLUTION_H
