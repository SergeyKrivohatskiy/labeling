#ifndef SIZE
#define SIZE
#include "point.h"

namespace geom2
{
    template<class T>
    struct size
    {
        T w;
        T h;
        operator point<T>() const;
        size operator+(const size &other) const;
        size operator-(const size &other) const;
        size& operator+=(const size &other);
        size& operator-=(const size &other);
    };

    template<class T>
    size<T>::operator point<T>() const
    {
        return point<T>(w, h);
    }

    template<class T>
    size<T> size<T>::operator+(const size &other) const
    {
        return size{w + other.w,h + other.h};
    }
    template<class T>
    size<T> size<T>::operator-(const size &other) const
    {
        return size{w - other.w, h - other.h};
    }
    template<class T>
    size<T>& size<T>::operator+=(const size &other)
    {
        h += other.h;
        w += other.w;
        return *this;
    }
    template<class T>
    size<T>& size<T>::operator-=(const size &other)
    {
        h -= other.h;
        w -= other.w;
        return *this;
    }

    typedef size<int> size_i;
    typedef size<float> size_f;
    typedef size<double> size_d;
} // namespace geom2
#endif // SIZE

