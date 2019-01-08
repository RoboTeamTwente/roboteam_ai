#include "CanShootModule.h"
#include "roboteam_utils/world_analysis.h"
#include "roboteam_utils/Position.h"
#include "roboteam_utils/Section.h"

namespace rtt {
namespace ai {
namespace dangerfinder {

CanShootModule::CanShootModule(double danger)
        :DangerModule(danger) { }

bool facingGoal(rtt::Position pos) {
    auto geom = rtt::ai::Field::get_field();

    rtt::Section goalSection{
            - geom.field_length/2, geom.goal_width/2,
            - geom.field_length/2, - geom.goal_width/2
    };
    rtt::Vector2 longVec{100, 0};
    longVec = longVec.rotate(pos.rot);
    longVec = longVec + pos.location();
    rtt::Vector2 isect = goalSection.intersection({pos.x, pos.y, longVec.x, longVec.y});
    return goalSection.pointOnLine(isect);
}

PartialResult CanShootModule::calculate(const roboteam_msgs::WorldRobot &bot, const roboteam_msgs::World &world) {
    rtt::Vector2 botPos(bot.pos);
    rtt::Vector2 goalPos = rtt::LastWorld::get_our_goal_center();

    auto obstacles = getObstaclesBetweenPoints(botPos, goalPos);

    bool hasObstacles = obstacles.size() != 0;
    bool hasBall = rtt::bot_has_ball(bot, world.ball);
    bool faceGoal = facingGoal({bot.pos.x, bot.pos.y, bot.angle});

    if (! hasObstacles && hasBall && faceGoal) {
        return {danger, DANGER_CAN_SHOOT};
    }
    return {};
}

} // dangerfinder
} // ai
} // rtt
