//
// Created by emiel on 24-4-18.
//

#include "WillReceiveBallModule.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {
namespace ai {
namespace dangerfinder {

REGISTER_MODULE("WillReceiveBall", WillReceiveBallModule);

WillReceiveBallModule::WillReceiveBallModule() : DangerModule("WillReceiveBall"){

}

PartialResult WillReceiveBallModule::calculate(
        const roboteam_msgs::WorldRobot &bot,
        const roboteam_msgs::World &world
){
    rtt::Vector2 velBot(bot.vel);
    rtt::Vector2 velBall(world.ball.vel);

    return {0, DANGER_NONE};
}

} // dangerfinder
} // ai
} // rtt
