#ifndef SIMPLE_OPTIMIZER_H
#define SIMPLE_OPTIMIZER_H

#include "positions_optimizer.h"
#include <set>

namespace labeling
{
    class simple_optimizer : public positions_optimizer
    {
    public:
        simple_optimizer();
        ~simple_optimizer();
        void register_label(screen_point_feature *);
        void unregister_label(screen_point_feature *);

        void register_obstacle(screen_obstacle *);
        void unregister_obstacle(screen_obstacle *);

        void best_fit(float time_max);
    private:
        typedef std::vector<screen_point_feature*> points_list_t;
        typedef std::vector<geom2::point_i> state_t;
        typedef std::pair<size_t, geom2::point_i> dstate_t;
    private:
        points_list_t points_list;
        state_t old_positions;
    private:
        state_t init_state() const;
        double dmetric(const state_t &state, const dstate_t &d_state) const;
        double calc_metric(const state_t &state, const dstate_t &d_state) const;
        void apply_state(const state_t &state);
    private:
        static bool do_jump(double t, double d_metrics);
        static double get_new_t(int iterations);
        static dstate_t update_state(const state_t &state);

        // returns closest distance between point and points
        // or double max value if points.size() == 0
        static double point_to_points_metric(const geom2::point_i &point,
                                             const geom2::points_i_list &points);
    };
} // namespace labeling
#endif // SIMPLE_OPTIMIZER_H
