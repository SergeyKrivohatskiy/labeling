#ifndef SCREEN_POINT_FEATURE_H
#define SCREEN_POINT_FEATURE_H
#include "geometry.h"

namespace labeling
{

    /*
     * Screen point interface
     *
     * You should implement this interface for your screen objects
     * if you want to use positions_optimizer to optimize labels
     * positioning
     *
     * @see positions_optimizer
     */
    class screen_point_feature
    {
    public:
        virtual ~screen_point_feature() {}

        /*
         * @return absolute position of this screen point
         */
        virtual const geom2::point_i& get_screen_pivot() const = 0;
        virtual const geom2::size_i& get_label_size() const = 0;

        virtual const geom2::point_i& get_label_offset() const = 0;
        virtual void set_label_offset(geom2::point_i const &) = 0;

        virtual const geom2::points_i_list& labels_best_positions() const = 0;
        virtual const geom2::points_i_list& labels_good_positions() const = 0;
    };
} // namespace labeling
#endif // SCREEN_POINT_FEATURE_H
