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

inline bool inside(
    const Vector2d& point,
    const physical_object& object)
{
    auto intersections = 0;

    const Vector2d ray_dir = { 1, 0 };

    const auto x2 = point.x() + ray_dir.x() * 100000.0;
    const auto y2 = point.y() + ray_dir.y() * 100000.0;

    for (const auto& line : object.lines())
    {
        const auto line_start_index = std::get<0>(line);
        const auto line_end_index = std::get<1>(line);

        const auto line_start_point = object.points()[line_start_index];
        const auto line_end_point = object.points()[line_end_index];

        const auto line_start = object.position() + std::get<0>(line_start_point);
        const auto line_end = object.position() + std::get<0>(line_end_point);

        double ix, iy;
        if (!lineSegmentIntersection(
            point.x(), point.y(), 
            x2, y2, 
            line_start.x(), line_start.y(), 
            line_end.x(), line_end.y(), 
            ix, iy))
            continue;

        ++intersections;
    }

    return intersections % 2 == 1;
}

inline void find_intersections(physical_object& object_a, physical_object& object_b, std::vector<contact_info>& contacts)
{
    const auto position_a = object_a.position();
    const auto position_b = object_b.position();

    for (const auto& point_a : object_a.points())
    {
        const auto point_pos = position_a + std::get<0>(point_a);

        if (!inside(point_pos, object_b))
            continue;

        line_t nearest_line;
        auto distance = std::numeric_limits<double>::max();
        auto penetration_depth = 0.0;
        Vector2d point_of_intersection;

        for (const auto& line : object_b.lines())
        {
            const auto line_start_index = std::get<0>(line);
            const auto line_end_index = std::get<1>(line);

            const auto& line_start_point = object_b.points()[line_start_index];
            const auto& line_end_point = object_b.points()[line_end_index];

            const auto line_start = position_b + std::get<0>(line_start_point);
            const auto line_end = position_b + std::get<0>(line_end_point);

            double ix, iy;
            if (!lineSegmentIntersection(
                position_a.x(), position_a.y(), 
                point_pos.x(), point_pos.y(), 
                line_start.x(), line_start.y(), 
                line_end.x(), line_end.y(), 
                ix, iy))
                continue;

            const auto n = ::normal(line_start, line_end);

            const auto current_distance =
                (Vector2d(point_pos.x(), point_pos.y()) - Vector2d(ix, iy)).dot(n);

            if (std::abs(current_distance) >= distance)
                continue;

            distance = current_distance;
            nearest_line = line;
            penetration_depth = current_distance;
            point_of_intersection = Vector2d(ix, iy);
        }

        contacts.emplace_back(object_a, point_a, object_b, nearest_line, penetration_depth, point_of_intersection);
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
    std::vector<contact_info> global_contacts;

    #pragma omp parallel shared(global_contacts)
    {
        #pragma omp for
        for (auto i = 0; i < polygons.size(); ++i)
        {
            polygons[i].clear_contacts();
        }

        #pragma omp for schedule(dynamic)
        for (auto i = 0; i < polygons.size(); ++i)
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

                #pragma omp critical
                {
                    polygon_a.add_contacts(local_contacts);
                    polygon_b.add_contacts(local_contacts);

                    std::move(
                        local_contacts.begin(),
                        local_contacts.end(),
                        std::back_inserter(global_contacts));
                }
            }
        }
    }

    return global_contacts;
}
#endif // COLLISION_DETECTION_H
