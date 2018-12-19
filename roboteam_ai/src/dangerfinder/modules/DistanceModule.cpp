#include "DistanceModule.h"

namespace rtt {
namespace ai {
namespace dangerfinder {

DistanceModule::DistanceModule(double danger)
        :DangerModule(danger) { }

inline Vector2 getGoalCenter() {
    return Vector2(Field::get_field().field_length/2, 0);
}

PartialResult
DistanceModule::calculate(const roboteam_msgs::WorldRobot &bot, const roboteam_msgs::World &world) {
    double dist = Vector2(bot.pos).dist(getGoalCenter());
    double last = lastDistances[bot.id];

    DangerFlag flag;
    lastDistances[bot.id] = dist;

    if (last != 0.0 && last < dist) {
        flag = DANGER_CLOSING;
    }
    else {
        flag = DANGER_NONE;
    }
    return {dist - danger, flag};
}

} // dangerfinder
} // ai
} // rtt
