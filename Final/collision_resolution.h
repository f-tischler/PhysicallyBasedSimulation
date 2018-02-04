#ifndef COLLISION_RESOLUTION_H
#define COLLISION_RESOLUTION_H

#include "ContactInfo.hpp"

/**
 * \brief Calculates impulse norm J as shown in lecture
 * \param a 
 * \param b 
 * \param offset_a 
 * \param offset_b 
 * \param normal 
 * \param rel_v_n 
 * \param restitution 
 * \param num_contacts 
 * \return Impulse norm
 */
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

/**
 * \brief Calculates the friction impulse using the tangential vector.
 *        Friction impulse norm is calculated the same way as the 
 *        normal impulse norm but using the tangent instead of the normal.
 *        Distinguishes between static and dynamic friction
 * \param a 
 * \param b 
 * \param offset_a 
 * \param offset_b 
 * \param normal 
 * \param rel_v 
 * \param rel_v_n 
 * \param j 
 * \param num_contacts 
 * \return 
 */
inline Vector2d calc_friction_impulse(
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

    if (std::abs(jt) < 0.000001) return { 0, 0 };

    const auto static_friction = 0.61;
    const auto dynamic_friction = 0.47;

    return std::abs(jt) < j * static_friction
        ? (tangent * jt).eval()
        : (tangent * -j * dynamic_friction).eval();
}

/**
 * \brief Applies the contact impulse to the objects
 * \param a 
 * \param b 
 * \param offset_a 
 * \param offset_b 
 * \param impulse 
 */
inline void apply_impulse(
    physical_object& a, 
    physical_object& b, 
    const Vector2d& offset_a, 
    const Vector2d& offset_b,
    const Vector2d& impulse)
{
    a.apply_impulse(-impulse, offset_a);
    b.apply_impulse( impulse, offset_b);
}

/**
 * \brief Applies normal and friction impulses to every object in the scene
 *        Uses a fixed restitution (0.3) if relative velocity is above a certain threshold
 *        (0 otherwise) to reduce jitter (restitution slop). 
 *        Skips resolution if objects moving apart
 * \param contacts 
 * \param dt 
 */
inline void collision_resolution(const std::vector<contact_info>& contacts, const double dt)
{
    #pragma omp parallel for
    for (auto i = 0; i < contacts.size(); ++i)
    {
        const auto& contact = contacts[i];

        auto& a = contact.line_owner();
        auto& b = contact.point_owner();

        const auto n = contact.normal();

        // find the number of contacts between these two objects
        const auto contact_points = std::count_if(contacts.begin(), contacts.end(),
            [&b, &a](const contact_info& c)
        {
            return  &c.point_owner() == &b && &c.line_owner() == &a ||
                &c.point_owner() == &a && &c.line_owner() == &b;
        });

        assert(contact_points > 0);

        // compute relative velocity 
        const auto rel_v = contact.relative_velocity();
        const auto rel_v_n = rel_v.dot(n);

        // do not apply impulses to objects
        // which a parting
        if (rel_v_n > 0)
            continue;

        // apply restitution slop
        const auto restitution_slop_sq = 0.5 * 0.5;
        const auto e = rel_v.squaredNorm() < restitution_slop_sq
            ? 0
            : 0.3;

        // normal impulse
        const auto j = calc_impulse_norm(a, b,
            contact.line_offset(),
            contact.contact_point_offset(),
            n,
            rel_v_n,
            e,
            contact_points);

        assert(std::abs(j) < 100000);

        const auto normal_impulse = j * n;

        const auto friction_impulse = calc_friction_impulse(a, b,
            contact.line_offset(),
            contact.contact_point_offset(),
            n, rel_v, rel_v_n, j,
            contact_points);

        #pragma omp critical
        {
            apply_impulse(a, b,
                contact.line_offset(),
                contact.contact_point_offset(),
                normal_impulse);

            
            apply_impulse(a, b,
                contact.line_offset(),
                contact.contact_point_offset(),
                friction_impulse);
        }
    }
}

/**
 * \brief Correct positions according to penetration depth
 *        to avoid objects sinking into each other. Also
 *        apply a slop here to avoid very small corrections
 * \param contacts 
 */
inline void correct_positions(const std::vector<contact_info>& contacts)
{
    #pragma omp parallel for
    for (auto i = 0; i < contacts.size(); ++i)
    {
        const auto& contact = contacts[i];

        auto& a = contact.line_owner();
        auto& b = contact.point_owner();

        const auto n = contact.normal();

        const auto percent = 0.3;   // usually 20% to 80%
        const auto slop = 0.05;     // usually 0.01 to 0.1

        // find the number of contacts between these two objects
        const auto contact_points = std::count_if(contacts.begin(), contacts.end(),
            [&b, &a](const contact_info& c)
        {
            return  &c.point_owner() == &b && &c.line_owner() == &a ||
                &c.point_owner() == &a && &c.line_owner() == &b;
        });

        const Vector2d correction = std::max(contact.penetration_depth() - slop, 0.0)
            / (a.inverse_mass() + b.inverse_mass()) * percent * n;

        #pragma omp critical
        {
            a.move(-a.inverse_mass() * correction / contact_points);
            b.move(b.inverse_mass() * correction / contact_points);
        }
    }
}
#endif // COLLISION_RESOLUTION_H
