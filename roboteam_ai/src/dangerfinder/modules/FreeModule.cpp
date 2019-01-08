#include "FreeModule.h"
#include "roboteam_utils/world_analysis.h"

namespace rtt {
namespace ai {
namespace dangerfinder {

FreeModule::FreeModule(double danger)
        :DangerModule(danger) { }

PartialResult FreeModule::calculate(const roboteam_msgs::WorldRobot &bot, const roboteam_msgs::World &world) {
    Vector2 ballPos(world.ball.pos);
    Vector2 botPos(bot.pos);
    const auto obstacles = getObstaclesBetweenPoints(botPos, ballPos, &rtt::ai::World::get_world());
    if (obstacles.empty()) {
        return {danger, DANGER_FREE};
    }
    return {};
}

} // dangerfinder
} // ai
} // rtt
