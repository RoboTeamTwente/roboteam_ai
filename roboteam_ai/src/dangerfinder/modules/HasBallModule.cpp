#include "HasBallModule.h"
#include "roboteam_utils/world_analysis.h"

namespace rtt {
namespace ai {
namespace dangerfinder {

HasBallModule::HasBallModule(double danger)
        :DangerModule(danger) { }

PartialResult HasBallModule::calculate(const roboteam_msgs::WorldRobot &bot, const roboteam_msgs::World &world) {
    bool hasBall = bot_has_ball(bot, world.ball);
    return {hasBall ? danger : 0, hasBall ? DANGER_HAS_BALL : DANGER_NONE};
}

} // dangerfinder
} // ai
} // rtt
