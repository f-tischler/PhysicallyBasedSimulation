#include "ContactInfo.hpp"

inline void collision_resolution(const std::vector<contact_info>& contacts, const double dt)
{
    for (const auto& contact : contacts)
    {
        const auto line_start = std::get<0>(std::get<0>(contact.line));
        const auto line_end = std::get<0>(std::get<1>(contact.line));

        const auto n = normal(line_start, line_end);

        auto& a = contact.line_owner;
        auto& b = contact.point_owner;

        const auto contact_points = std::count_if(contacts.begin(), contacts.end(),
            [&b, &a](const contact_info& c)
        {
            return &c.point_owner == &b && &c.line_owner == &a ||
                &c.point_owner == &a && &c.line_owner == &b;
        });

        const auto b_point_offset = std::get<0>(contact.point);
        const auto b_point_velocity = std::get<1>(contact.point);

		auto tmp = std::get<0>(contact.line);
		auto tmp2 = std::get<1>(contact.line);
        const Vector2d a_point_offset = (std::get<0>(tmp) + std::get<0>(tmp2)) / 2.0;
        const Vector2d a_point_velocity = (std::get<1>(tmp) + std::get<1>(tmp2)) / 2.0;

        const Vector2d rv = b_point_velocity - a_point_velocity;
        const auto relative_velocity = rv.dot(n);

        if (relative_velocity > 0)
            continue;

        const auto e = rv.squaredNorm() < (gravity * dt).squaredNorm() + 0.01
            ? 0
            : 0.3;

        const auto ra_n = cross2(a_point_offset, n);
        const auto rb_n = cross2(b_point_offset, n);

        const auto t_a = a.inverse_mass() + a.inverse_inertia() * ra_n * ra_n;
        const auto t_b = b.inverse_mass() + b.inverse_inertia() * rb_n * rb_n;
        const auto denom = t_a + t_b;

        const auto j = -(1 + e) * relative_velocity / denom / contact_points;

        const auto normal_impulse = j * n;

        a.apply_impulse(-normal_impulse, a_point_offset);
        b.apply_impulse(normal_impulse, b_point_offset);

        // friction
        const auto tangent = (rv - n * rv.dot(n)).normalized();

        const auto jt = -rv.dot(tangent) / denom / contact_points;

        if (std::abs(jt) < 0.000001) continue;

        const auto static_friction = 0.61;
        const auto dynamic_friction = 0.47;

        const auto tangent_impulse = std::abs(jt) < j * static_friction
            ? (tangent * jt).eval()
            : (tangent * -j * dynamic_friction).eval();

        a.apply_impulse(-tangent_impulse, a_point_offset);
        b.apply_impulse(tangent_impulse, b_point_offset);
    }
}

inline void correct_positions(const std::vector<contact_info>& contacts)
{
    for (const auto& contact : contacts)
    {
        const auto line_start = std::get<0>(std::get<0>(contact.line));
        const auto line_end = std::get<0>(std::get<1>(contact.line));
        const auto n = normal(line_start, line_end);

        auto& a = contact.line_owner;
        auto& b = contact.point_owner;

        const auto percent = 0.6; // usually 20% to 80%
        const auto slop = 0.05; // usually 0.01 to 0.1

        const auto contact_points = std::count_if(contacts.begin(), contacts.end(),
            [&b, &a](const contact_info& c)
        {
            return &c.point_owner == &b && &c.line_owner == &a ||
                &c.point_owner == &a && &c.line_owner == &b;
        });

        const Vector2d correction = std::max(contact.penetration_depth - slop, 0.0)
            / (a.inverse_mass() + b.inverse_mass()) * percent * n ;

        a.move(-a.inverse_mass() * correction / contact_points);
        b.move(b.inverse_mass() * correction / contact_points);
    }
}
