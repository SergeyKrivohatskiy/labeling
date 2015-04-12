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
        point();
        //point(const point &); default
        point(T x, T y);
        T x;
        T y;
        //point& operator=(const point &other); default
        point operator+(const point &other) const;
        point operator-(const point &other) const;
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
    point<T> point<T>::operator+(const point &other) const
    {
        point new_point;
        new_point.x = x + other.x;
        new_point.y = y + other.y;
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
        return x * x + y * y;
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
    };

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

    // Rectangle sizes should be >= 0
    template<class T>
    T rectangle_intersection(const rectangle<T> &l, const rectangle<T> &r)
    {
        const T &x_top = std::max(l.left_bottom.x, r.left_bottom.x);
        const T &y_top = std::max(l.left_bottom.y, r.left_bottom.y);
        const T &x_bot = std::min(l.left_bottom.x + l.size.w, r.left_bottom.x + r.size.w);
        const T &y_bot = std::min(l.left_bottom.y + l.size.h, r.left_bottom.y + r.size.h);
        if(x_top >= x_bot || y_top >= y_bot)
        {
            return T();
        }

        return (x_top - x_bot) * (y_top - y_bot);
    }

    template<class T>
    double points_distance(const point<T> &l, const point<T> &r)
    {
        return (r - l).norm();
    }

    template<class T>
    T sqr_points_distance(const point<T> &l, const point<T> &r)
    {
        return (r - l).sqr_norm();
    }

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

} // namespace geom2


#endif // GEOMETRY

