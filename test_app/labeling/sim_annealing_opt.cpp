#include "sim_annealing_opt.h"
#include "utils.h"
#include <chrono>
#include <math.h>
#include <random>
#include <algorithm>
#include <limits>
#ifdef _DEBUG
#include <QtDebug>
static double METRIC_CHANGE_SUMM = 0;
static double FITS_COUNT = 0;
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

    void sim_annealing_opt::best_fit(float time_max)
    {
        auto start = high_resolution_clock::now();

        state_t state = init_state();
        if(!state.size())
        {
            return;
        }
        std::vector<double> metrics = init_metric(state);

#ifdef _DEBUG
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

            // This is not accurate d_metric calculation. But it works too
            // Accurate calculation is "calc_metric(,, d_state.second) -
            // calc_metric(,,zero_offset)"
            double d_metric =
                    calc_metric(state, d_state.first, d_state.second) -
                    metrics[d_state.first];
            if(d_metric < 0 || do_jump(t, d_metric))
            {
#ifdef _DEBUG
                metric_change += d_metric;
#endif
                metrics[d_state.first] += d_metric;
                state[d_state.first] += d_state.second;
            }
            iterations += 1;
            t = get_new_t(iterations);
            current_time =
                    (duration_cast<milliseconds>(
                         high_resolution_clock::now() - start)).count();
        } while(t > 0 &&
                current_time < time_max &&
                iterations < max_iterations);


        apply_state(state);
#ifdef _DEBUG
        METRIC_CHANGE_SUMM += metric_change;
        FITS_COUNT += 1;
        qDebug() << metric_change << " metric_change";
        qDebug() << iterations << " iterations";
        qDebug() << METRIC_CHANGE_SUMM / FITS_COUNT << " average change";
#endif
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

    double sim_annealing_opt::calc_metric(const state_t &state,
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

    double sim_annealing_opt::point_to_points_metric(
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
