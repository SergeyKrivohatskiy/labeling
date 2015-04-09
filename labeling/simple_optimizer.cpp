#include "simple_optimizer.h"
#include <chrono>
#include <math.h>
#include <random>
#include <limits>
#include <QtDebug>


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

    void simple_optimizer::register_obstacle(screen_obstacle *)
    {
        // TODO
    }

    void simple_optimizer::unregister_obstacle(screen_obstacle *)
    {
        // TODO
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
        point_i d_pos = point_i(rand() % 10 - 5, rand() % 10 - 5);
        return dstate_t(idx, d_pos);
    }

    double simple_optimizer::dmetric(const state_t &state, const dstate_t &d_state) const
    {
        size_t i = d_state.first;
        return calc_metric(state, dstate_t(i, state[i] + d_state.second)) -
                calc_metric(state, dstate_t(i, state[i]));
    }

    double simple_optimizer::calc_metric(const state_t &state, const dstate_t &d_state) const
    {
        double summ = 0;

        size_t i = d_state.first;
        const point_i &new_offset = d_state.second;
        const screen_point_feature *point_ptr1 = points_list[i];

        double d_offset = sqr_points_distance(new_offset, old_positions[i]);
        if (d_offset != 0)
        {
            summ += 10 * d_offset + 30;
        }

        double best_pos_penalty =
                2 * point_to_points_metric(
                    new_offset, point_ptr1->labels_best_positions());
        double good_pos_penalty =
                1 * point_to_points_metric(
                    new_offset, point_ptr1->labels_good_positions());
        double min_penalty = min(best_pos_penalty, good_pos_penalty);
        min_penalty = min_penalty == double_limits::max() ? 0 : min_penalty;
        if (min_penalty != 0) {
            summ += min_penalty;
            summ += 100;
        }

        double labels_intersection = 0;
        for(size_t j = 0; j < state.size(); ++j)
        {
            if(i == j)
            {
                continue;
            }
            rectangle_i new_rect1 =
                {new_offset + point_ptr1->get_screen_pivot(),
                 point_ptr1->get_label_size()};
            rectangle_i new_rect2 =
                {state[j] + points_list[j]->get_screen_pivot(),
                 points_list[j]->get_label_size()};

            labels_intersection += rectangle_intersection(new_rect1, new_rect2);
        }

        if(labels_intersection != 0)
        {
            summ += labels_intersection;
            summ += 300;
        }
        return summ;
    }

    void simple_optimizer::best_fit(float time_max)
    {
        double t = 1;
        auto start = high_resolution_clock::now();

        state_t state = init_state();

        int iterations = 0;
        int64_t current_time;
        do
        {
            dstate_t d_state = update_state(state);

            double d_metric = dmetric(state, d_state);
            if(d_metric < 0 || do_jump(t, d_metric))
            {
                state[d_state.first] = state[d_state.first] + d_state.second;
            }
            t = get_new_t(iterations);
            iterations += 1;
            current_time = (duration_cast<milliseconds>(high_resolution_clock::now() - start)).count();
        } while(t > 0 && current_time < time_max);


        apply_state(state);
        qDebug() << iterations << " iterations";
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
