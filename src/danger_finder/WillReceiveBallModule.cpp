//
// Created by emiel on 24-4-18.
//
// thought out but not implemented yet, Math should be ready (ask Emiel)
#include "roboteam_world/danger_finder/WillReceiveBallModule.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {
    namespace df {

        REGISTER_MODULE("WillReceiveBall", WillReceiveBallModule);

        WillReceiveBallModule::WillReceiveBallModule() : DangerModule("WillReceiveBall"){

        }

        PartialResult WillReceiveBallModule::calculate(
                const roboteam_msgs::WorldRobot &bot,
                const roboteam_msgs::World &world
        ){
            Vector2 velBot(bot.vel);
            Vector2 velBall(world.ball.vel);

            return {0, DANGER_NONE};
        }



    }
}