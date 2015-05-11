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
        void apply_state(const state_t &state);
        state_t init_state();
        points_list_t::iterator move_fixed_to_end();
    protected:
        points_list_t points_list;
        obstacles_list_t obstacles_list;
    };
} // namespace labeling
#endif // BASE_OPTIMIZER_H
