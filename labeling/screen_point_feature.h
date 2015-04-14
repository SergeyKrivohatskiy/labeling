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
        /*
         * Prefered position is a pair containing double priority and
         * point relative to screen_pivot
         * Prioriry might be from 0 to +inf
         * ~1.0 - normal priority
         */
        typedef std::pair<double, geom2::point_i> prevered_position;
        typedef std::vector<prevered_position> prevered_pos_list;
    public:
        virtual ~screen_point_feature() {}

        /*
         * @return absolute position of this screen point
         */
        virtual const geom2::point_i& get_screen_pivot() const = 0;
        virtual const geom2::size_i& get_label_size() const = 0;

        /*
         * Current position of the label left bottom point relative
         * to screen_pivot
         */
        virtual const geom2::point_i& get_label_offset() const = 0;
        virtual void set_label_offset(geom2::point_i const &) = 0;

        /*
         * If label is fixed it won't be moved by position optimizer
         *
         * @return true if label is fixed
         */
        virtual bool is_label_fixed() const = 0;

        /*
         * @return a list of prefered positions. The empty list is
         * same(for positions optimizer) to list containing one
         * item == prevered_position(1.0, point_i{0, 0})
         */
        virtual const prevered_pos_list& get_prefered_positions() const = 0;
    };
} // namespace labeling
#endif // SCREEN_POINT_FEATURE_H
