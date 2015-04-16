#ifndef SEGMENT
#define SEGMENT
#include "point.h"

namespace geom2
{
    template<class T>
    struct segment
    {
        point<T> start;
        point<T> end;
    };

    typedef segment<int> segment_i;
    typedef segment<float> segment_f;
    typedef segment<double> segment_d;
} // namespace geom2
#endif // SEGMENT

