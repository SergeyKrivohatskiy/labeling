#ifndef POINT
#define POINT
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
        return point(x + other.x,
                     y + other.y);
    }

    template<class T>
    point<T> point<T>::operator*(const T &v) const
    {
        return point(x * v,
                     y * v);
    }

    template<class T>
    point<T> point<T>::operator/(const T &v) const
    {
        return point(x / v,
                     y / v);
    }

    template<class T>
    point<T> point<T>::operator-(const point &other) const
    {
        return point(x - other.x,
                     y - other.y);
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

} // namespace geom2
#endif // POINT

