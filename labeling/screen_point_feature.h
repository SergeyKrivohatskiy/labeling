#ifndef SCREEN_POINT_FEATURE_H
#define SCREEN_POINT_FEATURE_H
#include "geometry.h"

namespace labeling
{

    class screen_point_feature
    {
    public:
        virtual ~screen_point_feature() {}

        virtual const geom2::point_i& get_screen_pivot() const = 0;
        virtual const geom2::size_i& get_label_size() const = 0;

        virtual const geom2::rectangle_i get_label_rect() const;

        virtual const geom2::point_i& get_label_offset() const = 0;
        virtual void set_label_offset(geom2::point_i const &) = 0;

        virtual const geom2::points_i_list& labels_best_positions() const = 0;
        virtual const geom2::points_i_list& labels_good_positions() const = 0;
    };

    int labels_intersection(const screen_point_feature &l, const screen_point_feature &r);
} // namespace labeling
#endif // SCREEN_POINT_FEATURE_H
