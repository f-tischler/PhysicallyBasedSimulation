#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

#include "physical_object.h"
#include "ContactInfo.hpp"
#include "Polygon.h"

inline bool lineSegmentIntersection(
    double Ax, double Ay,
    double Bx, double By,
    double Cx, double Cy,
    double Dx, double Dy,
    double& X, double& Y) {

    double  distAB, theCos, theSin, newX, ABpos;

    //  Fail if either line segment is zero-length.
    if (Ax == Bx && Ay == By || Cx == Dx && Cy == Dy) return false;

    //  Fail if the segments share an end-point.
    if (Ax == Cx && Ay == Cy || Bx == Cx && By == Cy
        || Ax == Dx && Ay == Dy || Bx == Dx && By == Dy) {
        return false;
    }

    //  (1) Translate the system so that point A is on the origin.
    Bx -= Ax; By -= Ay;
    Cx -= Ax; Cy -= Ay;
    Dx -= Ax; Dy -= Ay;

    //  Discover the length of segment A-B.
    distAB = sqrt(Bx*Bx + By * By);

    //  (2) Rotate the system so that point B is on the positive X axis.
    theCos = Bx / distAB;
    theSin = By / distAB;
    newX = Cx * theCos + Cy * theSin;
    Cy = Cy * theCos - Cx * theSin; Cx = newX;
    newX = Dx * theCos + Dy * theSin;
    Dy = Dy * theCos - Dx * theSin; Dx = newX;

    //  Fail if segment C-D doesn't cross line A-B.
    if (Cy<0. && Dy<0. || Cy >= 0. && Dy >= 0.) return false;

    //  (3) Discover the position of the intersection point along line A-B.
    ABpos = Dx + (Cx - Dx)*Dy / (Dy - Cy);

    //  Fail if segment C-D crosses line A-B outside of segment A-B.
    if (ABpos<0. || ABpos>distAB) return false;

    //  (4) Apply the discovered position to line A-B in the original coordinate system.
    X = Ax + ABpos * theCos;
    Y = Ay + ABpos * theSin;

    //  Success.
    return true;
}

inline bool inside(const Vector2d point,
    const Vector2d point_offset,
    const std::vector<line_t>& lines,
    const Vector2d line_offset)
{
    auto intersections = 0;

    for (const auto& line_b : lines)
    {
        const auto x1 = point_offset.x() + point.x();
        const auto y1 = point_offset.y() + point.y();

        const auto x2 = point_offset.x() + point.x() * 100000.0;
        const auto y2 = point_offset.y() + point.y() * 100000.0;

        const auto x3 = line_offset.x() + std::get<0>(std::get<0>(line_b)).x();
        const auto y3 = line_offset.y() + std::get<0>(std::get<0>(line_b)).y();

        const auto x4 = line_offset.x() + std::get<0>(std::get<1>(line_b)).x();
        const auto y4 = line_offset.y() + std::get<0>(std::get<1>(line_b)).y();

        double ix, iy;
        if (!lineSegmentIntersection(x1, y1, x2, y2, x3, y3, x4, y4, ix, iy))
            continue;

        ++intersections;
    }

    return intersections % 2 == 1;
}

inline void find_intersections(physical_object& object_a, physical_object& object_b, std::vector<contact_info>& contacts)
{
    const auto get_lines = [](const physical_object& object)
    {
        std::vector<line_t> lines;

        const auto& points = object.points();

        for (size_t i = 0; i < points.size(); i++)
        {
            auto end_index = i == points.size() - 1
                ? 0
                : i + 1;

            lines.emplace_back(points[i], points[end_index]);
        }

        return lines;
    };

    const auto points_a = object_a.points();
    const auto lines_b = get_lines(object_b);

    const auto position_a = object_a.position();
    const auto position_b = object_b.position();

    for (const auto& point_a : points_a)
    {
        const auto point = std::get<0>(point_a);

        if (!inside(point, position_a, lines_b, position_b))
            continue;

        line_t line;
        auto distance = std::numeric_limits<double>::max();
        auto penetration_depth = 0.0;
        Vector2d point_of_intersection;

        for (const auto& line_b : lines_b)
        {
            const auto line_start = position_b + std::get<0>(std::get<0>(line_b));
            const auto line_end = position_b + std::get<0>(std::get<1>(line_b));

            const auto x1 = position_a.x();
            const auto y1 = position_a.y();

            const auto x2 = position_a.x() + point.x();
            const auto y2 = position_a.y() + point.y();

            const auto x3 = line_start.x();
            const auto y3 = line_start.y();

            const auto x4 = line_end.x();
            const auto y4 = line_end.y();

            double ix, iy;
            if (!lineSegmentIntersection(x1, y1, x2, y2, x3, y3, x4, y4, ix, iy))
                continue;

            const auto current_distance =
                (Vector2d(x2, y2) - Vector2d(ix, iy)).norm();

            if (current_distance >= distance)
                continue;

            distance = current_distance;
            line = line_b;
            penetration_depth = current_distance;
            point_of_intersection = Vector2d(ix, iy);
        }

        contacts.emplace_back(object_a, point_a, object_b, line, penetration_depth, point_of_intersection);
    }
}

inline bool intersects(physical_object& a, physical_object& b, std::vector<contact_info>& contacts)
{
    if ((a.center_of_mass_global() -
        b.center_of_mass_global()).norm() >
        a.bounding_radius() +
        b.bounding_radius())
        return false;

    find_intersections(a, b, contacts);
    find_intersections(b, a, contacts);

    return !contacts.empty();
}

inline std::vector<contact_info> collision_detection(std::vector<polygon>& polygons)
{
    for (auto& polygon_a : polygons)
    {
        polygon_a.clear_contacts();
    }

    std::vector<contact_info> global_contacts;
    for (auto i = 0u; i < polygons.size(); ++i)
    {
        auto& polygon_a = polygons[i];

        for (auto j = i + 1; j < polygons.size(); ++j)
        {
            auto& polygon_b = polygons[j];

            auto& object_a = polygon_a.get_physical_object();
            auto& object_b = polygon_b.get_physical_object();

            if (object_a.get_type() == object_type::fixed &&
                object_b.get_type() == object_type::fixed)
                continue;

            std::vector<contact_info> local_contacts;
            if (!intersects(object_a, object_b, local_contacts))
                continue;

            polygon_a.add_contacts(local_contacts);
            polygon_b.add_contacts(local_contacts);

            std::move(
                local_contacts.begin(),
                local_contacts.end(),
                std::back_inserter(global_contacts));
        }
    }

    return global_contacts;
}
#endif // COLLISION_DETECTION_H
