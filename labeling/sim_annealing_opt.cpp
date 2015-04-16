#include "sim_annealing_opt.h"
#include <chrono>
#include <math.h>
#include <random>
#include <limits>
#define DEBUG
#ifdef DEBUG
#include <QtDebug>
double METRIC_CHANGE_SUMM = 0;
double FITS_COUNT = 0;
#endif


using namespace geom2;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
using std::chrono::duration_cast;
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
    sim_annealing_opt::sim_annealing_opt()
    {}

    sim_annealing_opt::~sim_annealing_opt()
    {}

    void sim_annealing_opt::register_label(screen_point_feature *point_ptr)
    {
        points_list.push_back(point_ptr);
    }

    void sim_annealing_opt::unregister_label(screen_point_feature *point_ptr)
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

    void sim_annealing_opt::register_obstacle(screen_obstacle *obstacle_ptr)
    {
        obstacles_list.push_back(obstacle_ptr);
    }

    void sim_annealing_opt::unregister_obstacle(screen_obstacle *obstacle_ptr)
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


    sim_annealing_opt::state_t sim_annealing_opt::init_state() const
    {
        state_t state;
        // removing const. Points reordering do not changes optimizer state
        points_list_t &p_list = const_cast<points_list_t&>(points_list);
        auto fixed_beg = p_list.begin();
        // Move point with fixed labels to the end
        for(auto it = p_list.begin(); it != p_list.end(); ++it)
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

    std::vector<double> sim_annealing_opt::init_metric(const state_t &state)
    {
        std::vector<double> metrics(state.size());
        point_i zero_offset;
        for(size_t i = 0; i < state.size(); ++i)
        {
            metrics[i] = calc_metric(state, i, zero_offset);
        }
        return metrics;
    }

    void sim_annealing_opt::apply_state(const state_t &state)
    {
        for(size_t i = 0; i < state.size(); ++i)
        {
            points_list[i]->set_label_offset(state[i]);
        }
    }

    double sim_annealing_opt::get_new_t(int iterations)
    {
        return 1.0 / iterations / iterations;
    }

    bool sim_annealing_opt::do_jump(double t, double d_metrics)
    {
        return rand() < (RAND_MAX * exp(-d_metrics / t));
    }

    sim_annealing_opt::dstate_t sim_annealing_opt::update_state(
            const state_t &state)
    {
        size_t idx = rand() % state.size();
        const size_i &label_size = points_list[idx]->get_label_size();
        int w = label_size.w / STATE_CHANGE_FACTOR + 1;
        int h = label_size.h / STATE_CHANGE_FACTOR + 1;
        int dx, dy;
        do
        {
            dx = rand() % (2 * w + 1) - w;
            dy = rand() % (2 * h + 1) - h;
        } while(!dx && ! dy);
        point_i d_pos = point_i(dx, dy);
        return dstate_t(idx, d_pos);
    }

    double sim_annealing_opt::calc_metric(const state_t &state,
                                         size_t i,
                                         const point_i &offset_change) const
    {
        double summ = 0;

        const screen_point_feature *point_ptr1 = points_list[i];
        point_i new_offset = state[i] + offset_change;

        double d_offset =
                sqr_points_distance(new_offset, point_ptr1->get_label_offset());
        summ += OFFSET_FACTOR * d_offset;

        summ += PREFERED_POSITIONS_PENALTY * point_to_points_metric(
                    new_offset, point_ptr1->get_prefered_positions());

        double labels_intersection = 0;
        rectangle_i new_rect1 =
            {new_offset + point_ptr1->get_screen_pivot(),
             point_ptr1->get_label_size()};
        for(size_t j = 0; j < state.size(); ++j)
        {
            if(i == j)
            {
                continue;
            }
            rectangle_i new_rect2 =
                {state[j] + points_list[j]->get_screen_pivot(),
                 points_list[j]->get_label_size()};
            labels_intersection += rectangle_intersection(new_rect1, new_rect2);
        }
        for(size_t j = state.size(); j < points_list.size(); ++j)
        {
            rectangle_i new_rect2 =
                {points_list[j]->get_screen_pivot() +
                 points_list[j]->get_label_offset(),
                 points_list[j]->get_label_size()};
            labels_intersection += rectangle_intersection(new_rect1, new_rect2);
        }
        summ += LABELS_INTERSECTION_PENALTY * labels_intersection;

        double obstacles_intersection = 0;
        for(screen_obstacle *obstacle_ptr: obstacles_list)
        {
            switch (obstacle_ptr->get_type()) {
            case screen_obstacle::box:
                obstacles_intersection +=
                        rectangle_intersection(new_rect1,
                                               *(obstacle_ptr->get_box()));
                break;
            case screen_obstacle::segment:
                obstacles_intersection +=
                        get_sqr_seg_rect_intersection(
                            *(obstacle_ptr->get_segment()),
                            new_rect1);
                break;
            }
        }
        summ += OBSTACLES_INTERSECTION_PENALTY * obstacles_intersection;

        return summ;
    }

    void sim_annealing_opt::best_fit(float time_max)
    {
        auto start = high_resolution_clock::now();

        state_t state = init_state();
        // there is no not fixed points
        // nothing to optimize
        if(!state.size())
        {
            return;
        }
        std::vector<double> metrics = init_metric(state);

#ifdef DEBUG
        double metric_change = 0;
#endif
        double t = 1;
        int iterations = 0;
        int max_iterations =
                MAX_ITERATIONS_FACTOR * static_cast<int>(state.size());
        int64_t current_time;
        do
        {
            dstate_t d_state = update_state(state);

            double d_metric =
                    calc_metric(state, d_state.first, d_state.second) -
                    metrics[d_state.first];
            if(d_metric < 0 || do_jump(t, d_metric))
            {
#ifdef DEBUG
                metric_change += d_metric;
#endif
                metrics[d_state.first] += d_metric;
                state[d_state.first] += d_state.second;
            }
            t = get_new_t(iterations);
            iterations += 1;
            current_time =
                    (duration_cast<milliseconds>(
                         high_resolution_clock::now() - start)).count();
        } while(t > 0 &&
                current_time < time_max &&
                iterations < max_iterations);


        apply_state(state);
#ifdef DEBUG
        METRIC_CHANGE_SUMM += metric_change;
        FITS_COUNT += 1;
        qDebug() << metric_change << " metric_change";
        qDebug() << iterations << " iterations";
        qDebug() << METRIC_CHANGE_SUMM / FITS_COUNT << " average change";
#endif
    }

    double sim_annealing_opt::point_to_points_metric(
            const point_i &point,
            const screen_point_feature::prefered_pos_list &points)
    {
        if(points.size() == 0)
        {
            return 2 * sqr_points_distance(point, point_i());
        }
        double min_distance = double_limits::max();
        double max_distance = double_limits::min();
        for(const screen_point_feature::prefered_position &second_point: points)
        {
            double cur_distance =
                    second_point.first *
                    sqr_points_distance(point, second_point.second);
            min_distance = min(min_distance, cur_distance);
            max_distance = std::max(max_distance, cur_distance);
        }
        return min_distance;
    }
} // namespace labeling
