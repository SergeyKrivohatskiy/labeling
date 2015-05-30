#include "ray_intersection_opt.h"
#include <chrono>
#include <limits>
#include <deque>
#define _USE_MATH_DEFINES
#include <math.h>
#ifdef _DEBUG
#include <QtDebug>
#include <assert.h>
#endif

using namespace geom2;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
using std::chrono::duration_cast;
using std::swap;
//TODO find out is it ok to do typedef's like this?
typedef labeling::screen_point_feature::prefered_position prefered_position;
typedef labeling::screen_point_feature::prefered_pos_list prefered_pos_list;

namespace labeling
{
    static const int RAYS_COUNT = 8;
    static const int RAYS_LENGTH = 14;
    static const int SQR_MAX_DIST_FROM_BEST = 100*100;
} // namespace labeling

namespace labeling
{
    ray_intersection_opt::ray_intersection_opt()
    {}

    ray_intersection_opt::~ray_intersection_opt()
    {}

    point_i ray_intersection_opt::rays_to_best_pos(size_t idx, const rays_list_t &rays)
    {
        point_i where_min;
        int min_sqr_distance = std::numeric_limits<int>::max();
        for(const ray_t &ray: rays)
        {
            point_i closest;
            int distance =
                    point_seg_sqr_distance(
                        points_list[idx]->get_prefered_positions()[0].second +
                    points_list[idx]->get_screen_pivot(), ray, &closest);
            if(distance < min_sqr_distance)
            {
                min_sqr_distance = distance;
                where_min = closest;
            }
        }

        return where_min;
    }

    void ray_intersection_opt::find_best_ray(
            const std::vector<rays_list_t> &points_rays,
            size_t in_process_count,
            size_t &idx_max_min,
            point_i &best_pos)
    {
        double max_min_available_space = -1;
        for(size_t idx = 0; idx < points_rays.size(); ++idx)
        {
            const rays_list_t &rays = points_rays[idx];
            point_i old_offset = points_list[idx]->get_label_offset();
            point_i where_min = rays_to_best_pos(idx, rays);
            points_list[idx]->set_label_offset(
                        where_min - points_list[idx]->get_screen_pivot());

            std::vector<rays_list_t> new_points_rays =
                    get_points_rays(in_process_count);

            double min_available_space = std::numeric_limits<double>::max();
            for(const rays_list_t &rays: new_points_rays)
            {
                double available_space = 0;
                for(const ray_t &ray: rays)
                {
                    // TODO change to integral e^(-dist)
                    available_space += points_distance(ray.start, ray.end);
                }
                if(available_space < min_available_space)
                {
                    min_available_space = available_space;
                }
            }

            if(min_available_space > max_min_available_space)
            {
                max_min_available_space = min_available_space;
                idx_max_min = idx;
                best_pos = where_min;
            }
            points_list[idx]->set_label_offset(old_offset);
        }
    }

    std::vector<ray_intersection_opt::rays_list_t>
        ray_intersection_opt::get_points_rays(size_t points_to_locate)
    {
        std::vector<rays_list_t> points_rays(points_to_locate);
        for(size_t idx = 0; idx < points_to_locate; ++idx)
        {
            points_rays[idx] =
                    available_positions(idx);
        }
        return points_rays;
    }

    void ray_intersection_opt::best_fit(float /*time_max*/)
    {
        auto fixed_begin = move_fixed_to_end();
        size_t in_process_count = fixed_begin - points_list.begin();

        while(in_process_count)
        {
            std::vector<rays_list_t> points_rays =
                    get_points_rays(in_process_count);
            size_t points_to_locate = in_process_count;
            for(size_t idx = 0; idx < points_to_locate;)
            {
                if (points_rays[idx].empty())
                {
                    points_to_locate -= 1;
                    points_rays[idx] = std::move(points_rays[points_to_locate]);
                    points_rays.erase(points_rays.end() - 1);
                    swap(points_list[idx], points_list[points_to_locate]);
                } else {
                    idx += 1;
                }
            }
            points_rays.resize(points_to_locate);

            if(points_rays.empty())
            {
                break;
            }

            size_t idx;
            point_i best_pos;
            find_best_ray(points_rays, in_process_count, idx, best_pos);

            point_i new_offset = best_pos - points_list[idx]->get_screen_pivot();
            points_list[idx]->set_label_offset(new_offset);
            in_process_count -= 1;
            swap(points_list[idx], points_list[in_process_count]);
        }



#ifdef _DEBUG
//        in_process_count - amount of points that is not located
        qDebug() << "labeled: " <<
                    1.0 - in_process_count /
                    (double)(fixed_begin - points_list.begin());
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

    ray_intersection_opt::rays_list_t ray_intersection_opt::init_rays(
            const screen_point_feature *point) const
    {
        point_i cur_pos = point->get_label_offset() + point->get_screen_pivot();
        point_i best_pos = point->get_screen_pivot() +
                point->get_prefered_positions()[0].second;
        rays_list_t rays;
        for(int i = 0; i < RAYS_COUNT; ++i)
        {
            double deg = 2.0 * M_PI * i / RAYS_COUNT;
            point_i r(static_cast<int>(sin(deg) * RAYS_LENGTH),
                      static_cast<int>(cos(deg) * RAYS_LENGTH));
            point_i end = cur_pos + r;
            if(sqr_points_distance(end, best_pos) <= SQR_MAX_DIST_FROM_BEST)
            {
                rays.push_back(ray_t{cur_pos, end});
            }
        }
        return rays;
    }

    ray_intersection_opt::rays_list_t ray_intersection_opt::available_positions(
            size_t point_idx) const
    {

        rays_list_t rays = init_rays(points_list[point_idx]);

        const size_i &label_size = points_list[point_idx]->get_label_size();
        for(size_t j = 0; j < points_list.size(); ++j)
        {
            // remove segments from ray for label positions that
            // intersects with other labels
            if(point_idx == j)
            {
                continue;
            }
            rectangle_i mink_addition =
                {points_list[j]->get_screen_pivot() +
                 points_list[j]->get_label_offset() - label_size,
                 points_list[j]->get_label_size() + label_size};
            intersect_rays(mink_addition, rays);
        }


        return rays;
    }

} // namespace labeling

