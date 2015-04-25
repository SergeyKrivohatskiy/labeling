#include "base_optimizer.h"
#include "utils.h"
#include <algorithm>
#include <limits>

using namespace geom2;
using std::min;
typedef std::numeric_limits<double> double_limits;

namespace labeling
{
    /*
     * Correct values from 1 to MAX_INT
     * The bigger the less steps will be generated each iteration
     */
    const int STATE_CHANGE_FACTOR = 10;
    /*
     * Correct values from 1 to MAX_INT/max_points_count(to avoid an overflow)
     * Affects max iteration
     * (max_iteration = MAX_ITERATIONS_FACTOR * points_count)
     *
     * Here is a table of MAX_ITERATIONS_FACTOR affecting
     * optimization(in terms of average metric change) for points_count = 100
     *
     * MAX_ITERATIONS_FACTOR    optimization
     * 1                        20%
     * 5                        43%
     * 10                       61%
     * 30                       84%
     * 50                       91%
     * 70                       92%
     * 100                      95%
     * 200                      100%
     * 400                      101%
     */
    const int MAX_ITERATIONS_FACTOR = 100;
    /*
     * Correct values from 0 to +inf
     * Affect penalty for moving labels
     */
    const double OFFSET_FACTOR = 10;
    /*
     * Correct values from 0 to +inf
     * Affect penalty for label-label intersections
     */
    const double LABELS_INTERSECTION_PENALTY = 4;
    /*
     * Correct values from 0 to +inf
     * Affect penalty for label-obstacle intersections
     */
    const double OBSTACLES_INTERSECTION_PENALTY = 1;
    /*
     * Correct values from 0 to +inf
     * Affect penalty for label-prefered position weighted distances
     */
    const double PREFERED_POSITIONS_PENALTY = 5;
} // namespace labeling

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


    base_optimizer::state_t base_optimizer::init_state()
    {
        state_t state;
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

        state.reserve(fixed_beg - points_list.begin());
        for(auto it = points_list.begin(); it != fixed_beg; ++it)
        {
            state.push_back((*it)->get_label_offset());
        }
        return state;
    }

    std::vector<double> base_optimizer::init_metric(const state_t &state)
    {
        std::vector<double> metrics(state.size());
        point_i zero_offset;
        for(size_t i = 0; i < state.size(); ++i)
        {
            metrics[i] = calc_metric(state, i, zero_offset);
        }
        return metrics;
    }

    void base_optimizer::apply_state(const state_t &state)
    {
        for(size_t i = 0; i < state.size(); ++i)
        {
            points_list[i]->set_label_offset(state[i]);
        }
    }

    double base_optimizer::calc_metric(const state_t &state,
                                         size_t i,
                                         const point_i &offset_change) const
    {
        double summ = 0;

        const screen_point_feature *point = points_list[i];
        point_i new_offset = state[i] + offset_change;

        summ += OFFSET_FACTOR * sqr_points_distance(
                    new_offset, point->get_label_offset());

        summ += PREFERED_POSITIONS_PENALTY * point_to_points_metric(
                    new_offset, point->get_prefered_positions());

        rectangle_i label_rect =
            {new_offset + point->get_screen_pivot(),
             point->get_label_size()};
        double labels_intersection = 0;
        for(size_t j = 0; j < state.size(); ++j)
        {
            if(i == j)
            {
                continue;
            }
            rectangle_i label_rect2 =
                {state[j] + points_list[j]->get_screen_pivot(),
                 points_list[j]->get_label_size()};
            labels_intersection +=
                    rectangle_intersection(label_rect, label_rect2);
        }
        for(size_t j = state.size(); j < points_list.size(); ++j)
        {
            rectangle_i label_rect2 = to_label_rect(points_list[j]);
            labels_intersection += rectangle_intersection(label_rect, label_rect2);
        }
        summ += LABELS_INTERSECTION_PENALTY * labels_intersection;

        double obstacles_intersection = 0;
        for(screen_obstacle *obstacle_ptr: obstacles_list)
        {
            switch (obstacle_ptr->get_type()) {
            case screen_obstacle::box:
                obstacles_intersection +=
                        rectangle_intersection(label_rect,
                                               *(obstacle_ptr->get_box()));
                break;
            case screen_obstacle::segment:
                obstacles_intersection +=
                        get_sqr_seg_rect_intersection(
                            *(obstacle_ptr->get_segment()),
                            label_rect);
                break;
            }
        }
        summ += OBSTACLES_INTERSECTION_PENALTY * obstacles_intersection;

        return summ;
    }

    double base_optimizer::point_to_points_metric(
            const point_i &point,
            const screen_point_feature::prefered_pos_list &points)
    {
        if(points.size() == 0)
        {
            return sqr_points_distance(point, point_i());
        }
        double min_distance = double_limits::max();
        for(const screen_point_feature::prefered_position &second_point: points)
        {
            double cur_distance =
                    second_point.first *
                    sqr_points_distance(point, second_point.second);
            min_distance = min(min_distance, cur_distance);
        }
        return min_distance;
    }
} // namespace labeling
