#ifndef BASE_SCREEN_OBSTACLE_H
#define BASE_SCREEN_OBSTACLE_H

#include "labeling/screen_obstacle.h"

namespace labeling
{
    class base_screen_obstacle : public screen_obstacle
    {
    public:
        base_screen_obstacle(const geom2::rectangle_i &box);
        base_screen_obstacle(const geom2::segment_i &segment);

        ~base_screen_obstacle() = default;

        type get_type() const;

        const geom2::rectangle_i * get_box() const;
        const geom2::segment_i * get_segment() const;
    private:
        geom2::rectangle_i box;
        geom2::segment_i segment;
        type t;
    };

} // namespace labeling
#endif // BASE_SCREEN_OBSTACLE_H
