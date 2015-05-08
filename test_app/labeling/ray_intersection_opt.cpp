#include "ray_intersection_opt.h"
#include <chrono>
#include <limits>
#include <deque>
#define _USE_MATH_DEFINES
#include <math.h>
#ifdef _DEBUG
#include <QtDebug>
#endif

using namespace geom2;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
using std::chrono::duration_cast;
//TODO find out is it ok to do typedef's like this?
typedef labeling::screen_point_feature::prefered_position prefered_position;
typedef labeling::screen_point_feature::prefered_pos_list prefered_pos_list;

namespace labeling
{
    static const int RAYS_COUNT = 12;
    static const int RAYS_LENGTH = 3;
    static const int MAX_DISTANCE_FROM_PREF = 10 * 10;
} // namespace labeling

namespace labeling
{
    ray_intersection_opt::ray_intersection_opt()
    {}

    ray_intersection_opt::~ray_intersection_opt()
    {}

    void ray_intersection_opt::best_fit(float time_max)
    {
        auto start = high_resolution_clock::now();
        state_t state = init_state();
        if(!state.size())
        {
            return;
        }

#ifdef _DEBUG
        int changed_count = 0;
#endif
        int iterations = 0;
        int64_t current_time;
        size_t current_idx = 0;
        bool changed = false;
        do
        {
            bool current_changed = fit_point(state, current_idx);
            changed |= current_changed;
#ifdef _DEBUG
            changed_count += current_changed ? 1 : 0;
#endif

            current_idx += 1;
            iterations += 1;
            if(current_idx == state.size())
            {
                if(!changed)
                {
                    break;
                }
                current_idx = 0;
            }
            current_time =
                    (duration_cast<milliseconds>(
                         high_resolution_clock::now() - start)).count();
        } while(current_time < time_max);

        apply_state(state);
#ifdef _DEBUG
        qDebug() << ((double)changed_count / iterations) << "changed_ratio";
        qDebug() << iterations << "iterations";
        qDebug() << ((double)iterations / state.size())
                 << "avg iterations per point";
#endif
    }

    void ray_intersection_opt::intersect_rays(const rectangle_i & mink_addition,
                                              rays_list_t &rays)
    {
        rays_list_t available;

        for(const ray_t &ray: rays)
        {
            bool start_in_rect =
                    point_in_rect(ray.start, mink_addition);
            bool end_in_rect =
                    point_in_rect(ray.end, mink_addition);
            if(start_in_rect && end_in_rect)
            {
                // ray of unavailable points
                continue;
            }
            point_i first_intersection;
            point_i second_intersection;
            double t1;
            double t2;
            int intersections =
                    seg_rect_intersection(ray, mink_addition,
                                          &first_intersection,
                                          &second_intersection,
                                          &t1, &t2);
            if(intersections == 0)
            {
                // all ray contains available_points
                available.push_back(ray);
                continue;
            }
            if(intersections == 1)
            {
                if(start_in_rect)
                {
                    available.push_back(segment_i{first_intersection,
                                                  ray.end});
                } else {
                    available.push_back(segment_i{ray.start,
                                                  first_intersection});
                }
                continue;
            }
            if(intersections == 2)
            {
                if(t1 < t2)
                {
                    available.push_back(segment_i{ray.start,
                                                  first_intersection});
                    available.push_back(segment_i{second_intersection,
                                                  ray.end});
                } else {
                    available.push_back(segment_i{ray.start,
                                                  second_intersection});
                    available.push_back(segment_i{first_intersection,
                                                  ray.end});
                }
            }
        }

        rays = std::move(available);
    }

    ray_intersection_opt::rays_list_t ray_intersection_opt::available_positions(
            state_t &state, size_t point_idx, const point_i &point) const
    {
        const size_i &label_size = points_list[point_idx]->get_label_size();

        rays_list_t rays;
        for(int i = 0; i < RAYS_COUNT; ++i)
        {
            double deg = 2.0 * M_PI * i / RAYS_COUNT;
            point_i r(static_cast<int>(sin(deg) * RAYS_LENGTH),
                      static_cast<int>(cos(deg) * RAYS_LENGTH));
            rays.push_back(ray_t{point, point + r});
        }

        for(size_t j = 0; j < state.size(); ++j)
        {
            // remove segments from ray for label positions that
            // intersects with other labels

            if(point_idx == j)
            {
                continue;
            }
            rectangle_i mink_addition =
                {state[j] + points_list[j]->get_screen_pivot() - label_size,
                 points_list[j]->get_label_size() + label_size};
            intersect_rays(mink_addition, rays);
        }
        for(size_t j = state.size(); j < points_list.size(); ++j)
        {
            rectangle_i mink_addition =
                {points_list[j]->get_label_offset() +
                 points_list[j]->get_screen_pivot() - label_size,
                 points_list[j]->get_label_size() + label_size};
            intersect_rays(mink_addition, rays);
        }

        return rays;
    }

    /*
     * Changes the position of specified by idx point using ray intersection
     * algorithm
     * @return true if state[point_idx] was changed
     */
    bool ray_intersection_opt::fit_point(state_t &state, size_t point_idx) const
    {
        const screen_point_feature *point = points_list[point_idx];

        point_i cur_pos =
                point->get_screen_pivot() +
                point->get_label_offset();
        rays_list_t rays =
                available_positions(state, point_idx, cur_pos);

        point_i best_pos = point->get_prefered_positions()[0].second +
                point->get_screen_pivot();
        int min_sqr_distance = std::numeric_limits<int>::max();
        point_i where_min;
        for(const ray_t &ray: rays)
        {
            point_i closest;
            int distance = point_seg_sqr_distance(best_pos, ray, &closest);
            if(distance < min_sqr_distance)
            {
                min_sqr_distance = distance;
                where_min = closest;
            }
        }

        point_i old_state = state[point_idx];

        if(rays.empty() || min_sqr_distance > MAX_DISTANCE_FROM_PREF)
        {
            // Move to closest(in tersms of weights) prefered position
            point_i to_best = best_pos - cur_pos;
            double norm = to_best.norm();
            if(norm == 0)
            {
                return false;
            }
            if(norm > RAYS_LENGTH)
            {
                state[point_idx] = cur_pos + to_best * RAYS_LENGTH /
                        static_cast<int>(norm) - point->get_screen_pivot();
            } else {
                state[point_idx] = best_pos - point->get_screen_pivot();
            }
        } else {
            state[point_idx] = where_min - point->get_screen_pivot();
        }


        return old_state.x != state[point_idx].x ||
                old_state.y != state[point_idx].y;
    }

} // namespace labeling

