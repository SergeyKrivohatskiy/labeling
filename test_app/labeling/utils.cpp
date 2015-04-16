#include "utils.h"

namespace labeling {
    geom2::rectangle_i to_label_rect(const screen_point_feature *point)
    {
        return geom2::rectangle_i{point->get_screen_pivot() +
                    point->get_label_offset(),
                    point->get_label_size()};
    }
} // namespace labeling
