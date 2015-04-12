#include "simple_optimizer.h"
#include <chrono>
#include <math.h>
#include <random>
#include <limits>
#define DEBUG
#ifdef DEBUG
#include <QtDebug>
#endif


using namespace geom2;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
using std::chrono::duration_cast;
using std::min;
typedef std::numeric_limits<double> double_limits;

namespace labeling
{

    simple_optimizer::simple_optimizer()
    {}

    simple_optimizer::~simple_optimizer()
    {}

    void simple_optimizer::register_label(screen_point_feature *point_ptr)
    {
        points_list.push_back(point_ptr);
        old_positions.push_back(point_ptr->get_label_offset());
    }

    void simple_optimizer::unregister_label(screen_point_feature *)
    {
        // TODO
    }

    void simple_optimizer::register_obstacle(screen_obstacle *obstacle_ptr)
    {
        obstacles_list.push_back(obstacle_ptr);
    }

    void simple_optimizer::unregister_obstacle(screen_obstacle *obstacle_ptr)
    {
        obstacles_list.erase(
                    std::find(obstacles_list.begin(), obstacles_list.end(), obstacle_ptr));
    }


    simple_optimizer::state_t simple_optimizer::init_state() const
    {
        state_t state;
        state.reserve(points_list.size());
        for(auto point_ptr: points_list)
        {
            state.push_back(point_ptr->get_label_offset());
        }
        return state;
    }

    std::vector<double> simple_optimizer::init_metric(const state_t &state)
    {
        std::vector<double> metrics(state.size());
        point_i zero_offset;
        for(size_t i = 0; i < state.size(); ++i)
        {
            metrics[i] = calc_metric(state, i, zero_offset);
        }
        return metrics;
    }

    void simple_optimizer::apply_state(const state_t &state)
    {
        for(size_t i = 0; i < state.size(); ++i)
        {
            points_list[i]->set_label_offset(state[i]);
        }
        old_positions = state;
    }

    double simple_optimizer::get_new_t(int iterations)
    {
        return 1.0 / iterations / iterations;
    }

    bool simple_optimizer::do_jump(double t, double d_metrics)
    {
        return rand() < (RAND_MAX * exp(-d_metrics / t));
    }

    simple_optimizer::dstate_t simple_optimizer::update_state(const state_t &state)
    {
        size_t idx = rand() % state.size();
        const size_i &label_size = points_list[idx]->get_label_size();
        int w = label_size.w / 10 + 1;
        int h = label_size.h / 10 + 1;
        int dx, dy;
        do
        {
            dx = rand() % (2 * w + 1) - w;
            dy = rand() % (2 * h + 1) - h;
        } while(!dx && ! dy);
        point_i d_pos = point_i(dx, dy);
        return dstate_t(idx, d_pos);
    }

    double simple_optimizer::calc_metric(const state_t &state, size_t i, const point_i &offset_change) const
    {
        double summ = 0;

        const screen_point_feature *point_ptr1 = points_list[i];
        point_i new_offset = state[i] + offset_change;

        double d_offset = sqr_points_distance(new_offset, old_positions[i]);
        if (d_offset != 0)
        {
            summ += 10 * d_offset + 10;
        }

        double best_pos_penalty =
                point_to_points_metric(
                    new_offset, point_ptr1->labels_best_positions());
        double min_penalty = 0;
        if(best_pos_penalty != double_limits::max())
        {
            best_pos_penalty *= 5;
            min_penalty = best_pos_penalty;
        }
        double good_pos_penalty =
                point_to_points_metric(
                    new_offset, point_ptr1->labels_good_positions());
        if(good_pos_penalty != double_limits::max())
        {
            good_pos_penalty *= 1;
            min_penalty = min(best_pos_penalty, good_pos_penalty + 300);
        }
        if (min_penalty != 0) {
            summ += min_penalty + 100;
        }

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
        if(labels_intersection != 0)
        {
            summ += 4 * labels_intersection + 300;
        }

        double obstacles_intersection = 0;
        for(screen_obstacle *obstacle_ptr: obstacles_list)
        {
            switch (obstacle_ptr->get_type()) {
            case screen_obstacle::box:
                obstacles_intersection +=
                        rectangle_intersection(new_rect1, *(obstacle_ptr->get_box()));
                break;
            case screen_obstacle::segment:
                // TODO
                break;
            }
        }
        if(obstacles_intersection != 0)
        {
            summ += 1 * obstacles_intersection + 70;
        }

        return summ;
    }

    void simple_optimizer::best_fit(float time_max)
    {
        auto start = high_resolution_clock::now();

        state_t state = init_state();
        std::vector<double> metrics = init_metric(state);

#ifdef DEBUG
        double metric_change = 0;
#endif
        double t = 1;
        int iterations = 0;
        int64_t current_time;
        do
        {
            dstate_t d_state = update_state(state);

            double d_metric = calc_metric(state, d_state.first, d_state.second) - metrics[d_state.first];
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
            current_time = (duration_cast<milliseconds>(high_resolution_clock::now() - start)).count();
        } while(t > 0 && current_time < time_max);


        apply_state(state);
#ifdef DEBUG
        qDebug() << metric_change << " metric_change";
        qDebug() << iterations << " iterations";
#endif
    }

    double simple_optimizer::point_to_points_metric(
            const point_i &point, const points_i_list &points)
    {
        double min_distance = double_limits::max();
        for(const point_i &second_point: points)
        {
            double cur_distance = sqr_points_distance(point, second_point);
            min_distance = min(min_distance, cur_distance);
        }
        return min_distance;
    }
} // namespace labeling
