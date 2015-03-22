#ifndef SIMPLE_OPTIMIZER_H
#define SIMPLE_OPTIMIZER_H

#include "positions_optimizer.h"

namespace labeling
{
    class simple_optimizer : public positions_optimizer
    {
    public:
        simple_optimizer();
        ~simple_optimizer();
        void register_label(screen_point_feature *);
        void unregister_label(screen_point_feature *);

        void register_obstacle(screen_obstacle *);
        void unregister_obstacle(screen_obstacle *);

        void best_fit(float time_max);
    };
} // namespace labeling
#endif // SIMPLE_OPTIMIZER_H
