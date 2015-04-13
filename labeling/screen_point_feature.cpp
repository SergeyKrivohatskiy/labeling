#include "screen_point_feature.h"


namespace labeling
{
    const geom2::rectangle_i screen_point_feature::get_label_rect() const
    {
        geom2::rectangle_i rect;
        rect.left_bottom = get_label_offset() + get_screen_pivot();
        rect.size = get_label_size();
        return rect;
    }
} // namespace labeling
