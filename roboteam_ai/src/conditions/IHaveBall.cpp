//
// Created by rolf on 19-10-18.
//
#include "IHaveBall.hpp"

namespace rtt{
namespace ai{

    //TODO: Fix global namespacing

    IHaveBall::IHaveBall(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {

    }
    bt::Node::Status IHaveBall::Update() {
        //TODO:ROBOT_ID variable name on the blackboard is not the same on old blackboard as we are not using the nested blackboard calls anymore!! (Don't forget to change tests!)
        int me=blackboard->GetInt("ROBOT_ID");
        bool us=blackboard->GetBool("our_team");
        auto opt_bot=World::getRobotForId(me,us);
        if (!opt_bot) {
            return Status::Failure;
        }
        bool has = World::bot_has_ball(*opt_bot, World::getBall());
        auto res = has ? Status::Success : Status::Failure;

        return res;
    }

}
}