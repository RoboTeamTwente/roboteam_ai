//
// Created by mrlukasbos on 2-5-19.
//

#include "ballIsMoving.h"


namespace rtt {
    namespace ai {

        ballIsMoving::ballIsMoving(std::string name, bt::Blackboard::Ptr blackboard)
                :Condition(std::move(name), std::move(blackboard)) { };


        bt::Node::Status ballIsMoving::onUpdate() {
            Vector2 ballVel=ball->vel;
            bool ballIsLayingStill = ballVel.length() < Constants::BALL_STILL_VEL();

            if (ballIsLayingStill ){
                return Status::Failure;
            }
            return Status::Success;
        }

    } // ai
} // rtt
