#ifndef GEOMETRY
#define GEOMETRY
#include <vector>

namespace geom2
{
    template<class T>
    struct point
    {
        T x;
        T y;
    };

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
} // namespace geom2


#endif // GEOMETRY

