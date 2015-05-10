#ifndef SIM_ANNEALING_OPT_H
#define SIM_ANNEALING_OPT_H

#include "positions_optimizer.h"
#include "base_optimizer.h"

namespace labeling
{
    /*
     * Positions optimizer that uses simulated annealing to optimize
     * labels positions
     */
    class sim_annealing_opt : public base_optimizer
    {
    public:
        sim_annealing_opt();
        ~sim_annealing_opt();
        void best_fit(float time_max);
    private:
        typedef std::pair<size_t, geom2::point_i> dstate_t;
    private:
        dstate_t update_state(const state_t &state);
        double calc_metric(const state_t &state, size_t i,
                           const geom2::point_i &new_offset) const;
        std::vector<double> init_metric(const state_t &state);
    private:
        static bool do_jump(double t, double d_metrics);
        static double get_new_t(int iterations);
        static double point_to_points_metric(const geom2::point_i &point,
                      const screen_point_feature::prefered_pos_list &points);
    };
} // namespace labeling
#endif // SIM_ANNEALING_OPT_H
