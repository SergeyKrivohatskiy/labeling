#include "simple_optimizer.h"
#include <chrono>
#include <math.h>
#include <random>


using namespace geom2;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
using std::chrono::duration_cast;

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

    simple_optimizer::state_t simple_optimizer::update_state(const state_t &state)
    {
        state_t new_state(state);
        size_t idx = rand() % new_state.size();
        new_state[idx] = new_state[idx] + point_i(rand() % 20 - 10, rand() % 20 - 10);
        return new_state;
    }

    double simple_optimizer::metric() const
    {
        return metric(init_state());
    }

    double simple_optimizer::metric(const state_t &state) const
    {
        double summ = 0;
        for(size_t i = 0; i < state.size(); ++i)
        {
            const point_i &new_offset = state[i];
            const screen_point_feature *point_ptr1 = points_list[i];
            summ += 0.5 * points_distance(new_offset, old_positions[i]);
            summ += points_distance(new_offset, point_ptr1->labels_best_positions()[0]);
            for(const screen_point_feature *point_ptr2: points_list)
            {
                if(point_ptr2 == point_ptr1)
                {
                    continue;
                }
                rectangle_i new_rect  =
                    {new_offset + point_ptr1->get_screen_pivot(),
                     point_ptr1->get_label_size()};

                summ += rectangle_intersection(point_ptr2->get_label_rect(), new_rect);
            }
        }
        return summ;
    }

    void simple_optimizer::best_fit(float time_max)
    {
        double t = 1;
        auto start = high_resolution_clock::now();

        state_t state = init_state();

        double state_metric = metric(state);
        double min_metric = state_metric;
        state_t min_state = state;

        int iterations = 0;
        int64_t current_time;
        do
        {
            state_t new_state = update_state(state);

            double new_state_metric = metric(new_state);
            if(state_metric > new_state_metric || do_jump(t, new_state_metric - state_metric))
            {
                if(min_metric > new_state_metric)
                {
                    min_state = new_state;
                    min_metric = new_state_metric;
                }
                state = new_state;
                state_metric = new_state_metric;
            }
            t = get_new_t(iterations);
            iterations += 1;
            current_time = (duration_cast<milliseconds>(high_resolution_clock::now() - start)).count();
        } while(t > 0 && current_time < time_max);


        apply_state(min_state);
    }
} // namespace labeling
