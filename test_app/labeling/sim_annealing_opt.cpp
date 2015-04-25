#include "sim_annealing_opt.h"
#include <chrono>
#include <math.h>
#include <random>
#ifdef _DEBUG
#include <QtDebug>
static double METRIC_CHANGE_SUMM = 0;
static double FITS_COUNT = 0;
#endif


using namespace geom2;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
using std::chrono::duration_cast;

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
        // there is no not fixed points
        // nothing to optimize
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
} // namespace labeling
