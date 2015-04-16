#ifndef RECTANGLE
#define RECTANGLE
#include "point.h"
#include "size.h"

namespace geom2
{
    template<class T>
    struct rectangle
    {
        point<T> left_bottom;
        size<T> size;
        T area() const;
        point<T> center() const;
        point<T> left_up() const;
        point<T> right_up() const;
        point<T> right_bottom() const;
    };

    template<class T>
    T rectangle<T>::area() const
    {
        return size.h * size.w;
    }

    template<class T>
    point<T> rectangle<T>::center() const
    {
        return point<T>(left_bottom.x + size.w / 2,
                        left_bottom.y + size.h / 2);
    }

    template<class T>
    point<T> rectangle<T>::left_up() const
    {
        return point<T>(left_bottom.x,
                        left_bottom.y + size.h);
    }

    template<class T>
    point<T> rectangle<T>::right_up() const
    {
        return point<T>(left_bottom.x + size.w,
                        left_bottom.y + size.h);
    }

    template<class T>
    point<T> rectangle<T>::right_bottom() const
    {
        return point<T>(left_bottom.x + size.w,
                        left_bottom.y);
    }

    typedef rectangle<int> rectangle_i;
    typedef rectangle<float> rectangle_f;
    typedef rectangle<double> rectangle_d;
} // namespace geom2
#endif // RECTANGLE

