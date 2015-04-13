#include "base_screen_obstacle.h"

namespace labeling
{
    using namespace geom2;

    base_screen_obstacle::base_screen_obstacle(const rectangle_i &box)
        :
          box(box),
          t(type::box)
    {

    }

    base_screen_obstacle::base_screen_obstacle(const segment_i &segment)
        :
          segment(segment),
          t(type::segment)
    {

    }

    screen_obstacle::type base_screen_obstacle::get_type() const
    {
        return t;
    }

    const rectangle_i* base_screen_obstacle::get_box() const
    {
        return &box;
    }
    const segment_i* base_screen_obstacle::get_segment() const
    {
        return &segment;
    }

} // namespace labeling
