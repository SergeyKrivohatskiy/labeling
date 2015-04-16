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
    };

    typedef rectangle<int> rectangle_i;
    typedef rectangle<float> rectangle_f;
    typedef rectangle<double> rectangle_d;
} // namespace geom2
#endif // RECTANGLE

