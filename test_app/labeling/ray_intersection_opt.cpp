#include "ray_intersection_opt.h"
#include <chrono>

using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
using std::chrono::duration_cast;

namespace labeling
{
    ray_intersection_opt::ray_intersection_opt()
    {}

    ray_intersection_opt::~ray_intersection_opt()
    {}

    void ray_intersection_opt::register_label(screen_point_feature *point_ptr)
    {
        points_list.push_back(point_ptr);
    }

    void ray_intersection_opt::unregister_label(screen_point_feature *point_ptr)
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

    void ray_intersection_opt::register_obstacle(screen_obstacle *obstacle_ptr)
    {
        obstacles_list.push_back(obstacle_ptr);
    }

    void ray_intersection_opt::unregister_obstacle(screen_obstacle *obstacle_ptr)
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
    void ray_intersection_opt::best_fit(float time_max)
    {
        auto start = high_resolution_clock::now();
        int iterations = 0;
        int64_t current_time;
        do
        {
            iterations += 1;
            current_time =
                    (duration_cast<milliseconds>(
                         high_resolution_clock::now() - start)).count();
        } while(current_time < time_max);
    }
} // namespace labeling

