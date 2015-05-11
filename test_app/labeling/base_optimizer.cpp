#include "base_optimizer.h"

using namespace geom2;

namespace labeling
{
    base_optimizer::base_optimizer()
    {}

    base_optimizer::~base_optimizer()
    {}

    void base_optimizer::register_label(screen_point_feature *point_ptr)
    {
        points_list.push_back(point_ptr);
    }

    void base_optimizer::unregister_label(screen_point_feature *point_ptr)
    {
        auto pos = std::find(points_list.begin(),
               points_list.end(),
               point_ptr);
        if(pos == points_list.end())
        {
            return;
        }
        points_list.erase(pos);
    }

    void base_optimizer::register_obstacle(screen_obstacle *obstacle_ptr)
    {
        obstacles_list.push_back(obstacle_ptr);
    }

    void base_optimizer::unregister_obstacle(screen_obstacle *obstacle_ptr)
    {
        auto pos = std::find(obstacles_list.begin(),
                             obstacles_list.end(),
                             obstacle_ptr);
        if(pos == obstacles_list.end())
        {
            return;
        }
        obstacles_list.erase(pos);
    }

    base_optimizer::points_list_t::iterator base_optimizer::move_fixed_to_end()
    {
        auto fixed_beg = points_list.begin();
        // Move point with fixed labels to the end
        for(auto it = points_list.begin(); it != points_list.end(); ++it)
        {
            if (!(*it)->is_label_fixed()) {
                auto tmp = *fixed_beg;
                *fixed_beg = *it;
                *it = tmp;
                ++fixed_beg;
            }
        }
        return fixed_beg;
    }


    base_optimizer::state_t base_optimizer::init_state()
    {
        state_t state;
        auto fixed_beg = move_fixed_to_end();
        state.reserve(fixed_beg - points_list.begin());
        for(auto it = points_list.begin(); it != fixed_beg; ++it)
        {
            state.push_back((*it)->get_label_offset());
        }
        return state;
    }

    void base_optimizer::apply_state(const state_t &state)
    {
        for(size_t i = 0; i < state.size(); ++i)
        {
            points_list[i]->set_label_offset(state[i]);
        }
    }
} // namespace labeling
