#ifndef RAY_INTERSECTION_OPT_H
#define RAY_INTERSECTION_OPT_H

#include "positions_optimizer.h"

namespace labeling
{
    /*
     * Positions optimizer that uses ray intersection algorithm to optimize
     * labels positions
     */
    class ray_intersection_opt : public positions_optimizer
    {
    public:
        ray_intersection_opt();
        ~ray_intersection_opt();
        void register_label(screen_point_feature *);
        void unregister_label(screen_point_feature *);

        void register_obstacle(screen_obstacle *);
        void unregister_obstacle(screen_obstacle *);

        void best_fit(float time_max);
    private:
        typedef std::vector<screen_point_feature*> points_list_t;
        typedef std::vector<screen_obstacle*> obstacles_list_t;
    private:
        points_list_t points_list;
        obstacles_list_t obstacles_list;
    };
} // namespace labeling
#endif // RAY_INTERSECTION_OPT_H
