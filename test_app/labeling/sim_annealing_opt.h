#ifndef SIM_ANNEALING_OPT_H
#define SIM_ANNEALING_OPT_H

#include "positions_optimizer.h"
#include <set>

namespace labeling
{
    /*
     * Positions optimizer that uses simulated annealing to optimize
     * labels positions
     */
    class sim_annealing_opt : public positions_optimizer
    {
    public:
        sim_annealing_opt();
        ~sim_annealing_opt();
        void register_label(screen_point_feature *);
        void unregister_label(screen_point_feature *);

        void register_obstacle(screen_obstacle *);
        void unregister_obstacle(screen_obstacle *);

        void best_fit(float time_max);
    private:
        typedef std::vector<screen_point_feature*> points_list_t;
        typedef std::vector<geom2::point_i> state_t;
        typedef std::pair<size_t, geom2::point_i> dstate_t;
        typedef std::vector<screen_obstacle*> obstacles_list_t;
    private:
        double calc_metric(const state_t &state, size_t i,
                           const geom2::point_i &new_offset) const;
        void apply_state(const state_t &state);
        dstate_t update_state(const state_t &state);
        state_t init_state();
        std::vector<double> sim_annealing_opt::init_metric(
                const state_t &state);
    private:
        static bool do_jump(double t, double d_metrics);
        static double get_new_t(int iterations);
        static double point_to_points_metric(const geom2::point_i &point,
                      const screen_point_feature::prefered_pos_list &points);
    private:
        points_list_t points_list;
        obstacles_list_t obstacles_list;
    };
} // namespace labeling
#endif // SIM_ANNEALING_OPT_H
