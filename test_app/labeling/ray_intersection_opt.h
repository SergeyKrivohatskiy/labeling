#ifndef RAY_INTERSECTION_OPT_H
#define RAY_INTERSECTION_OPT_H

#include "positions_optimizer.h"
#include "base_optimizer.h"

namespace labeling
{
    /*
     * Positions optimizer that uses ray intersection algorithm to optimize
     * labels positions
     */
    class ray_intersection_opt : public base_optimizer
    {
    public:
        ray_intersection_opt();
        ~ray_intersection_opt();

        void best_fit(float time_max);
    private:
        typedef geom2::segment_i ray_t;
        typedef std::vector<ray_t> rays_list_t;
    private:
        bool fit_point(state_t &state, size_t point_idx) const;
        rays_list_t available_positions(state_t &state,
                                        size_t point_idx,
                                        const geom2::point_i &point) const;
    private:
        static void intersect_rays(const geom2::rectangle_i & mink_addition,
                                   rays_list_t &rays);
    };
} // namespace labeling
#endif // RAY_INTERSECTION_OPT_H
