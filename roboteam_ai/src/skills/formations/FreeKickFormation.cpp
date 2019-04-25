//
// Created by baris on 23-4-19.
//

#include "FreeKickFormation.h"
namespace rtt {
namespace ai {

std::vector<Vector2> FreeKickFormation::posses;
std::shared_ptr<vector<std::shared_ptr<rtt::ai::world::Robot>>> rtt::ai::FreeKickFormation::robotsInFormation = nullptr;


Vector2 FreeKickFormation::getFormationPosition() {

    update = true;
    posses = getFreeKickPositions(robotsInFormation->size());
    std::vector<int> robotIds;

    for (auto & i : *robotsInFormation) {
        robotIds.push_back(i->id);
    }

    rtt::HungarianAlgorithm hungarian;
    auto shortestDistances = hungarian.getRobotPositions(robotIds, true, posses);
    return shortestDistances[robot->id];
}
shared_ptr<vector<shared_ptr<bt::Leaf::Robot>>> FreeKickFormation::robotsInFormationPtr() {
    return robotsInFormation;
}

FreeKickFormation::FreeKickFormation(std::string name, bt::Blackboard::Ptr blackboard)
        :Formation(name, blackboard) {
    robotsInFormation = std::make_shared<vector<std::shared_ptr<world::Robot>>>();

}
void FreeKickFormation::onTerminate(Skill::Status s) {
    Formation::onTerminate(s);
    update = false;
}


    std::vector<Vector2> FreeKickFormation::getFreeKickPositions(int number) {
        // Two defenders, one robot to receive the ball, rest 3 in a diagonal
        auto lengthOffset = rtt::ai::world::field->get_field().field_length/4.0;
        auto widthOffset = rtt::ai::world::field->get_field().field_width/4.0;
        Vector2 penaltyUs = rtt::ai::world::field->getPenaltyPoint(true);
        Vector2 ballPos = rtt::ai::world::world->getBall()->pos;
        Vector2 penaltyThem = rtt::ai::world::field->getPenaltyPoint(false);
        int ballPosMultiplier = (ballPos.y >= 0 ? (- 1) : 1);
        Vector2 lineProgress = {- 0.4, 0};


        Vector2 def1 = {penaltyUs.x + lengthOffset/3.0, penaltyUs.y + widthOffset/1.5};
        Vector2 def2 = {penaltyUs.x + lengthOffset/3.0, - (penaltyUs.y + widthOffset/1.5)};


        Vector2 line1 = {penaltyThem.x - (lengthOffset/3.0), (penaltyThem.y + widthOffset)*ballPosMultiplier};
        Vector2 line2 = line1 + lineProgress;
        Vector2 line3 = line2 + lineProgress;

        std::vector<Vector2> temp = {line1, def1, def2, line2, line3};
        std::vector<Vector2> res;
        for (int i = 0; i < number; i ++) {
            res.emplace_back(temp.at(i));
        }
        return res;
    }


}

}
