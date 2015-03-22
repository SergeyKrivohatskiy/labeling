#ifndef POSITIONS_OPTIMIZER
#define POSITIONS_OPTIMIZER
#include "screen_point_feature.h"
#include "screen_obstacle.h"

namespace labeling
{
    class positions_optimizer
    {
    public:
        virtual ~positions_optimizer() {}

        virtual void register_label(screen_point_feature *) = 0;
        virtual void unregister_label(screen_point_feature *) = 0;

        virtual void register_obstacle(screen_obstacle *) = 0;
        virtual void unregister_obstacle(screen_obstacle *) = 0;

        virtual void best_fit(float time_max) = 0;
    };
} // namespace labeling

#endif // POSITIONS_OPTIMIZER

