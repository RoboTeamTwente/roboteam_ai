//
// Created by rolf on 19-10-18.
//
#include "IHaveBall.hpp"

namespace rtt {
namespace ai {

//TODO: Fix global namespacing
bool bot_has_ball(const roboteam_msgs::WorldRobot &bot, const roboteam_msgs::WorldBall &ball) {
    Vector2 ball_vec(ball.pos), bot_vec(bot.pos);
    Vector2 ball_norm = (ball_vec - bot_vec);

    double dist = ball_norm.length();
    double angle = ball_norm.angle();

    //TODO: Check if the angle taken this way does not fail because of angle jump at pi or 2 pi (it should)
    //TODO: TEST if this is from centre of dribbler of robot in practice. What does
    // Within 15 cm and .4 radians (of center of dribbler)
    return dist <= .15 && fabs(angle - bot.angle) <= .4;
}
IHaveBall::IHaveBall(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(name, blackboard) {

}
bt::Node::Status IHaveBall::Update() {
    //TODO:ROBOT_ID variable name on the blackboard is not the same on old blackboard as we are not using the nested blackboard calls anymore!! (Don't forget to change tests!)
    int me = blackboard->GetInt("ROBOT_ID");
    bool us = blackboard->GetBool("our_team");
    auto opt_bot = World::getRobotForId(me, us);
    if (! opt_bot) {
        return Status::Failure;
    }
    bool has = bot_has_ball(*opt_bot, World::getBall());
    auto res = has ? Status::Success : Status::Failure;

    return res;
}

}
}