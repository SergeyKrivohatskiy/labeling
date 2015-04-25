#ifndef BASE_OPTIMIZER_H
#define BASE_OPTIMIZER_H
#include "positions_optimizer.h"

namespace labeling
{
    class base_optimizer : public positions_optimizer
    {
    public:
        base_optimizer();
        ~base_optimizer();

        void register_label(screen_point_feature *);
        void unregister_label(screen_point_feature *);

        void register_obstacle(screen_obstacle *);
        void unregister_obstacle(screen_obstacle *);
    protected:
        typedef std::vector<screen_point_feature*> points_list_t;
        typedef std::vector<geom2::point_i> state_t;
        typedef std::vector<screen_obstacle*> obstacles_list_t;
    protected:
        double calc_metric(const state_t &state, size_t i,
                           const geom2::point_i &new_offset) const;
        void apply_state(const state_t &state);
        state_t init_state();
        std::vector<double> init_metric(const state_t &state);
    private:
        static double point_to_points_metric(const geom2::point_i &point,
                      const screen_point_feature::prefered_pos_list &points);
    protected:
        points_list_t points_list;
        obstacles_list_t obstacles_list;
    };
} // namespace labeling
#endif // BASE_OPTIMIZER_H
