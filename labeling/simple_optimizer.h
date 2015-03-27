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

        double metric() const;
    private:
        typedef std::set<screen_point_feature*> points_set_t;
        typedef std::pair<geom2::point_i, screen_point_feature*> new_pos_t;
        typedef std::vector<new_pos_t> state_t;
    private:
        points_set_t points_set;
    private:
        state_t init_state() const;
        double metric(const state_t &state) const;
        void apply_state(const state_t &state);
    private:
        static bool do_jump(double t, double d_metrics);
        static double get_new_t(double t);
        static state_t update_state(const state_t &state);
    };
} // namespace labeling
#endif // SIMPLE_OPTIMIZER_H
