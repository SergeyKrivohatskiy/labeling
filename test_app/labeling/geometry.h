#ifndef GEOMETRY
#define GEOMETRY
#include <vector>
#include <algorithm>
#include <math.h>
#include "point.h"
#include "segment.h"
#include "size.h"
#include "rectangle.h"

namespace geom2
{
    typedef std::vector<point_i> points_i_list;

    template<class T>
    bool value_in_range(const T &value, const T &l, const T &r)
    {
        return value >= l && value <= r;
    }

    template<class T>
    bool point_in_rect(const point<T> &p, const rectangle<T> &r)
    {
        return p.x >= r.left_bottom.x && p.y >= r.left_bottom.y &&
                p.x <= r.left_bottom.x + r.size.w &&
                p.y <= r.left_bottom.y + r.size.h;
    }

    /*
     * Calculates area of rectangles intersection
     * Rectangle sizes should be not negative(>= 0)
     */
    template<class T>
    T rectangle_intersection(const rectangle<T> &l, const rectangle<T> &r)
    {
        const T &x_top = std::max(l.left_bottom.x, r.left_bottom.x);
        const T &y_top = std::max(l.left_bottom.y, r.left_bottom.y);
        const T &x_bot = std::min(l.left_bottom.x + l.size.w,
                                  r.left_bottom.x + r.size.w);
        const T &y_bot = std::min(l.left_bottom.y + l.size.h,
                                  r.left_bottom.y + r.size.h);
        if(x_top >= x_bot || y_top >= y_bot)
        {
            return T();
        }

        return (x_top - x_bot) * (y_top - y_bot);
    }

    /*
     * Calculates ||l - r||
     */
    template<class T>
    double points_distance(const point<T> &l, const point<T> &r)
    {
        return (r - l).norm();
    }

    /*
     * Calculates ||l - r||^2
     */
    template<class T>
    T sqr_points_distance(const point<T> &l, const point<T> &r)
    {
        return (r - l).sqr_norm();
    }

    /*
     * Checks two segments for intersection
     *
     * @param intersection_point is unnecessary output parameter
     * Contains the intersection point if there is one and
     * intersection_point is not NULL
     * @return true if fst_seg intersects sec_seg
     */
    template<class T>
    bool segments_intersection(const segment<T> &fst_seg,
                               const segment<T> &sec_seg,
                               point<T> *intersection_point)
    {
        const point<T> &p = fst_seg.start;
        const point<T> &q = sec_seg.start;
        point<T> r = fst_seg.end - p;
        point<T> s = sec_seg.end - q;

        point<T> q_p = q - p;
        double rxs = r * s;
        double q_pxs = q_p * s;
        double q_pxr = q_p * r;

        if(rxs == 0)
        {
            if(q_pxr != 0)
            {
                // two segments are parallel and non-intersecting
                return false;
            }
            // TODO check for disjoint and overlapp?
            return false;
        }

        double t = q_pxs / rxs;
        double u = q_pxr / rxs;
        // TODO maybe add epsilon for value_in_range checking?
        if(!value_in_range(t, 0.0, 1.0) || !value_in_range(u, 0.0, 1.0))
        {
            // the two segments are not parallel and do not intersect
            return false;
        }
        if(intersection_point != nullptr)
        {
            *intersection_point = p +
                    point<T>(static_cast<T>(r.x * t),
                             static_cast<T>(r.y * t));
        }
        return true;
    }

    /*
     * Checks segment-rectangle intersection
     *
     * @param intersection_point1 is unnecessary output parameter
     * Contains the first intersection point if there is one and
     * intersection_point1 is not NULL
     * @param intersection_point2 is unnecessary output parameter
     * Contains the second intersection point if there is one and
     * intersection_point2 is not NULL
     * @return number of intersection points found(from 0 to 2)
     */
    template<class T>
    int seg_rect_intersection(const segment<T> &seg,
                               const rectangle<T> &rect,
                              point<T> *intersection_point1,
                              point<T> *intersection_point2)
    {
        segment<T> rect_seg[4] = {
            {rect.left_bottom,
             rect.left_bottom + point<T>(rect.size.w, 0)},
            {rect.left_bottom + point<T>(rect.size.h, 0),
             rect.left_bottom + rect.size},
            {rect.left_bottom + rect.size,
             rect.left_bottom + point<T>(0, rect.size.h)},
            {rect.left_bottom + point<T>(0, rect.size.h),
             rect.left_bottom}};
        int intersections = 0;
        point<T> intersection_point;
        for(int i = 0; i < 4; ++i)
        {
            if(segments_intersection(seg, rect_seg[i], &intersection_point))
            {
                intersections += 1;
                if(intersections == 2)
                {
                    if(intersection_point2 != nullptr)
                    {
                        *intersection_point2 = intersection_point;
                    }
                    return intersections;
                }
                if(intersection_point1 != nullptr)
                {
                    *intersection_point1 = intersection_point;
                }
            }
        }

        return intersections;
    }

    template<class T>
    T get_sqr_seg_rect_intersection(const segment<T> &seg,
                                    const rectangle<T> &rect)
    {
        point<T> intersection_point1;
        point<T> intersection_point2;
        int intersections =
                seg_rect_intersection(seg, rect,
                                      &intersection_point1,
                                      &intersection_point2);
        if(intersections == 0)
        {
            return T();
        }

        if(intersections == 1)
        {
            // one point inside rect
            if(point_in_rect(seg.start, rect))
            {
                return sqr_points_distance(seg.start, intersection_point1);
            } else {
                return sqr_points_distance(seg.end, intersection_point1);
            }
        }
        return sqr_points_distance(intersection_point1, intersection_point2);
    }

} // namespace geom2


#endif // GEOMETRY

