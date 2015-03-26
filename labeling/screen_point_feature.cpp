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


    int labels_intersection(const screen_point_feature &l, const screen_point_feature &r)
    {
        return geom2::rectangle_intersection(l.get_label_rect(), r.get_label_rect());
    }


} // namespace labeling
