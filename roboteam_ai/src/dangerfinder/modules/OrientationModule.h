//
// DangerModule which assigns higher scores to robots which are oriented towards our goal.
//

#ifndef ROBOTEAM_AI_DANGER_ORIENTATION_H
#define ROBOTEAM_AI_DANGER_ORIENTATION_H

#include "DangerModule.h"

namespace rtt {
namespace ai {
namespace dangerfinder {

class OrientationModule : public DangerModule {
    public:
        explicit OrientationModule() = default;
        OrientationModule(double factor, double scalar, double danger);
        PartialResult calculate(const roboteam_msgs::WorldRobot &bot, const roboteam_msgs::World &world) override;
    private:
        double factor = 0;
        double scalar = 0;
        double danger = 0;
};

} // dangerfinder
} // ai
} // rtt

#endif // ROBOTEAM_AI_DANGER_ORIENTATION_H