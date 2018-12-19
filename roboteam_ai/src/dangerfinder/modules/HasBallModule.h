// Dangermodule that returns a high danger score if the opponent robot has the ball
// it then also raises the DANGER_HAS_BALL flag


#ifndef ROBOTEAM_AI_DANGER_HASBALL_H
#define ROBOTEAM_AI_DANGER_HASBALL_H

#include "DangerModule.h"

namespace rtt {
namespace ai {
namespace dangerfinder {

class HasBallModule : public DangerModule {
    public:
        explicit HasBallModule() = default;
        explicit HasBallModule(double danger);
        PartialResult calculate(const roboteam_msgs::WorldRobot &bot, const roboteam_msgs::World &world) override;
};

} // dangerfinder
} // ai
} // rtt

#endif // ROBOTEAM_AI_DANGER_HASBALL_H
