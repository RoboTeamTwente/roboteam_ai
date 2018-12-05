/**
 * DangerModule which checks whether a robot has the ball as well as line of sight to our goal.
 * If so, it gives it a massive danger score and sets the DANGER_CAN_SHOOT flag.
 */

#ifndef ROBOTEAM_AI_CAN_SHOOT_MODULE_H
#define ROBOTEAM_AI_CAN_SHOOT_MODULE_H

#include "DangerModule.h"

namespace rtt {
namespace ai {
namespace dangerfinder {

class CanShootModule : public DangerModule {
    public:
        explicit CanShootModule() = default;
        explicit CanShootModule(double danger);
        PartialResult calculate(const roboteam_msgs::WorldRobot &bot, const roboteam_msgs::World &world) override;
};

} // dangerfinder
} // ai
} // rtt

#endif //ROBOTEAM_AI_CAN_SHOOT_MODULE_H