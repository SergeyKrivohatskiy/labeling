#ifndef GEOMETRY
#define GEOMETRY
#include <vector>
#include <algorithm>
#include <math.h>

namespace geom2
{
    template<class T>
    struct point
    {
        T x;
        T y;

        point();
        point(T x, T y);
        point operator+(const point &other) const;
        point operator-(const point &other) const;
        T operator*(const point &other) const;
        T dot(const point &other) const;
        point operator*(const T &v) const;
        point operator/(const T &v) const;
        point& operator+=(const point &other);
        point& operator-=(const point &other);
        double norm() const;
        T sqr_norm() const;
    };


    template<class T>
    point<T>::point()
        :
          x(),
          y()
    {
    }

    template<class T>
    point<T>::point(T x, T y)
        :
          x(x),
          y(y)
    {
    }

    template<class T>
    T point<T>::operator*(const point<T> &other) const
    {
        return x * other.y - y * other.x;
    }

    template<class T>
    T point<T>::dot(const point<T> &other) const
    {
        return x * other.x + y * other.y;
    }

    template<class T>
    point<T> point<T>::operator+(const point &other) const
    {
        point new_point;
        new_point.x = x + other.x;
        new_point.y = y + other.y;
        return new_point;
    }

    template<class T>
    point<T> point<T>::operator*(const T &v) const
    {
        point new_point;
        new_point.x = x * v;
        new_point.y = y * v;
        return new_point;
    }

    template<class T>
    point<T> point<T>::operator/(const T &v) const
    {
        point new_point;
        new_point.x = x / v;
        new_point.y = y / v;
        return new_point;
    }

    template<class T>
    point<T> point<T>::operator-(const point &other) const
    {
        point new_point;
        new_point.x = x - other.x;
        new_point.y = y - other.y;
        return new_point;
    }

    template<class T>
    point<T>& point<T>::operator+=(const point &other)
    {
        x += other.x;
        y += other.y;
        return *this;
    }

    template<class T>
    point<T>& point<T>::operator-=(const point &other)
    {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    template<class T>
    double point<T>::norm() const
    {
        return sqrt(sqr_norm());
    }

    template<class T>
    T point<T>::sqr_norm() const
    {
        return (*this).dot(*this);
    }

    typedef point<int> point_i;
    typedef point<float> point_f;
    typedef point<double> point_d;

    template<class T>
    struct segment
    {
        point<T> start;
        point<T> end;
    };

    typedef segment<int> segment_i;
    typedef segment<float> segment_f;
    typedef segment<double> segment_d;


    template<class T>
    struct size
    {
        T w;
        T h;
        operator point<T>() const;
    };

    template<class T>
    size<T>::operator point<T>() const
    {
        return point<T>(w, h);
    }

    typedef size<int> size_i;
    typedef size<float> size_f;
    typedef size<double> size_d;

    template<class T>
    struct rectangle
    {
        point<T> left_bottom;
        size<T> size;
    };

    typedef rectangle<int> rectangle_i;
    typedef rectangle<float> rectangle_f;
    typedef rectangle<double> rectangle_d;

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
     * Calculates cos of min angle betveen two radius(position) vectors
     */
    template<class T>
    double angle_cos(const point<T> &l, const point<T> &r)
    {
        T r_norm = r.norm();
        T l_norm = l.norm();
        if(!r_norm || !l_norm)
        {
            return 1.0;
        }
        return sqrt(l.x * r.x + l.y * r.y) / r_norm / l_norm;
    }

    /*
     * Calculates cos^2 of min angle betveen two radius(position) vectors
     */
    template<class T>
    double sqr_angle_cos(const point<T> &l, const point<T> &r)
    {
        T r_norm = r.sqr_norm();
        T l_norm = l.sqr_norm();
        if(!r_norm || !l_norm)
        {
            return 1.0;
        }
        return (l.x * r.x + l.y * r.y) / r_norm / l_norm;
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

