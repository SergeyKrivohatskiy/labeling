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
    };

    template<class T>
    size<T>::operator point<T>() const
    {
        return point<T>(w, h);
    }

    typedef size<int> size_i;
    typedef size<float> size_f;
    typedef size<double> size_d;
} // namespace geom2
#endif // SIZE

