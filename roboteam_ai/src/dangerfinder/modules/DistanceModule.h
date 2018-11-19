/**
 * \class DistanceModule
 * \brief A DangerModule which assigns higher scores to robots which are closer to our goal.
 * It may also set the DANGER_CLOSING flag if applicable.
 */

#ifndef ROBOTEAM_AI_DISTANCE_MODULE_H
#define ROBOTEAM_AI_DISTANCE_MODULE_H

#include "DangerModule.h"
#include <map>
#include "../../utilities/Field.h"

namespace rtt {
namespace ai {
namespace dangerfinder {

class DistanceModule : public DangerModule {
    public:
        explicit DistanceModule() = default;
        explicit DistanceModule(double danger);
        PartialResult calculate(const roboteam_msgs::WorldRobot &bot, const roboteam_msgs::World &world) override;
    private:
        std::map<int, double> lastDistances;
};

} // dangerfinder
} // ai
} // rtt

#endif // ROBOTEAM_AI_DISTANCE_MODULE_H