#ifndef SCREEN_OBSTACLE
#define SCREEN_OBSTACLE
#include "geometry.h"

namespace labeling
{
    class screen_obstacle
    {
    public:
        enum type
        {
            box,
            segment
        };
    public:
        virtual ~screen_obstacle() {}

        virtual type get_type() const = 0;

        virtual const geom2::rectangle_i* get_box() const = 0;
        virtual const geom2::segment_i* get_segment() const = 0;
    };

} // namespace labeling

#endif // SCREEN_OBSTACLE

