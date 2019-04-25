//
// Created by baris on 24-4-19.
//

#include "DefendFreeKick.h"
namespace rtt {
namespace ai {

std::vector<Vector2> DefendFreeKick::posses;
std::shared_ptr<vector<std::shared_ptr<rtt::ai::world::Robot>>> rtt::ai::DefendFreeKick::robotsInFormation = nullptr;


Vector2 DefendFreeKick::getFormationPosition() {
    gtp.setAvoidBall(0.55);

    update = true;
    posses = getDefendFreeKick(robotsInFormation->size());
    std::vector<int> robotIds;

    for (auto & i : *robotsInFormation) {
        robotIds.push_back(i->id);
    }

    rtt::HungarianAlgorithm hungarian;
    auto shortestDistances = hungarian.getRobotPositions(robotIds, true, posses);
    return shortestDistances[robot->id];
}
shared_ptr<vector<shared_ptr<bt::Leaf::Robot>>> DefendFreeKick::robotsInFormationPtr() {
    return robotsInFormation;
}

DefendFreeKick::DefendFreeKick(std::string name, bt::Blackboard::Ptr blackboard)
        :Formation(name, blackboard) {
    robotsInFormation = std::make_shared<vector<std::shared_ptr<world::Robot>>>();

}
void DefendFreeKick::onTerminate(Skill::Status s) {
    Formation::onTerminate(s);
    update = false;
}

std::vector<Vector2> DefendFreeKick::getDefendFreeKick(int number) {
    // makes a free kick line
    auto lengthOffset = rtt::ai::world::field->get_field().field_length/4.0;
    auto widthOffset = rtt::ai::world::field->get_field().field_width/4.0;
    Vector2 goalUS = rtt::ai::world::field->get_our_goal_center();
    Vector2 ballPos = rtt::ai::world::world->getBall()->pos;
    Vector2 penaltyUs = rtt::ai::world::field->getPenaltyPoint(true);

    Vector2 lineProgress = ((goalUS-ballPos).stretchToLength(0.28)).rotate(M_PI_2);
    Vector2 lineBegin = ballPos + (goalUS - ballPos).stretchToLength(0.75);

    Vector2 line2 = lineBegin + lineProgress;
    Vector2 line3 = lineBegin - lineProgress;

    Vector2 def1 = {penaltyUs.x + 0.12, penaltyUs.y + widthOffset/2.0};
    Vector2 def2 = {penaltyUs.x + 0.12, - (penaltyUs.y + widthOffset/2.0)};
    Vector2 def3 = def1 + (def2-def1).stretchToLength((def2-def1).length()/3.0);
    Vector2 def4 = (def2-def3).stretchToLength((def2-def3).length()/2.0) + def3;

    std::vector<Vector2> temp = {lineBegin, def1, line3, def2, line2, def3, def4};

    std::vector<Vector2> res;
    for (int i = 0; i < number; i ++) {
        res.emplace_back(temp.at(i));
    }
    return res;
}


}

}
