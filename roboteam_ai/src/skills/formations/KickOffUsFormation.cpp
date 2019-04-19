#include <roboteam_ai/src/analysis/DecisionMaker.h>
#include <roboteam_ai/src/analysis/GameAnalyzer.h>
#include "KickOffUsFormation.h"

namespace rtt {
namespace ai {

    std::shared_ptr<vector<std::shared_ptr<world::Robot>>> KickOffUsFormation::robotsInFormation = nullptr;

    KickOffUsFormation::KickOffUsFormation(std::string name, bt::Blackboard::Ptr blackboard)
        : Formation(name, blackboard) {
        robotsInFormation = std::make_shared<vector<std::shared_ptr<world::Robot>>>();
    }

Vector2 KickOffUsFormation::getFormationPosition() {
    std::vector<Vector2> targetLocations;
    std::vector<int> robotIds;
    auto field = world::field->get_field();
    double targetLocationX;
    rtt::ai::analysis::AnalysisReport report = * rtt::ai::analysis::GameAnalyzer::getInstance().getMostRecentReport();
    rtt::ai::analysis::BallPossession possession = report.ballPossession;
    analysis::DecisionMaker maker;
    analysis::PlayStyle style = maker.getRecommendedPlayStyle(possession);

    int def = style.amountOfDefenders;
    int mid = style.amountOfMidfielders;
    int att = style.amountOfAttackers;


    if ((def+mid+att) != static_cast<int>(robotsInFormation->size())) return {};

    int count = 0;


    // if no attackers and no midfielders, claim one defender for the middle dot to kickoff
    if (att == 0 && mid == 0 && def > 0) {
        targetLocations.emplace_back(-0.2, 0.0);
        robotIds.push_back(robotsInFormation->at(count)->id);
        count++;
        def -= 1;
    }
    // set up the defensive locations
    targetLocationX = - field.field_length/3;
    for (int i = 0; i < def; i++) {
        double targetLocationY = ((field.field_width/(def+1))*(i+1)) - field.field_width/2;
        targetLocations.emplace_back(targetLocationX, targetLocationY);

        if (static_cast<int>(robotsInFormation->size()) > count) {
            robotIds.push_back(robotsInFormation->at(count)->id);
            count++;
        }
    }

    // if no attackers but midfielder, claim one defender for the middle dot to kickoff
    if (att == 0 && mid > 0) {
        targetLocations.emplace_back(-0.2, 0.0);
        robotIds.push_back(robotsInFormation->at(count)->id);
        count++;
        mid -= 1;
    }
    // set up midfield locations
    targetLocationX = - field.field_length/5;
    for (int i = 0; i < mid; i++) {
        double targetLocationY = (((field.field_width/(mid+1))*(i+1)) - field.field_width/2) * 0.8;
        targetLocations.emplace_back(targetLocationX, targetLocationY);
        if (static_cast<int>(robotsInFormation->size()) > count) {
            robotIds.push_back(robotsInFormation->at(count)->id);
            count++;
        }
    }




    // if we have attackers we should claim one for the middle dot.
    if (att > 0) {
        targetLocations.emplace_back(-0.2, 0.0);
        robotIds.push_back(robotsInFormation->at(count)->id);
        count++;
        att -= 1;
    }

    // set up offensive locations
    targetLocationX = -field.field_length/12;
    // claim other attackers
    for (int i = 0; i < att; i++) {
        double targetLocationY = (((field.field_width/(att+1))*(i+1)) - field.field_width/2) * 1.5;
        targetLocations.emplace_back(targetLocationX, targetLocationY);
        if (static_cast<int>(robotsInFormation->size()) > count) {
            robotIds.push_back(robotsInFormation->at(count)->id);
            count++;
        }
    }

    rtt::HungarianAlgorithm hungarian;
    auto shortestDistances = hungarian.getRobotPositions(robotIds, true, targetLocations);
    return shortestDistances.at(robot->id);
}

shared_ptr<vector<shared_ptr<bt::Leaf::Robot>>> KickOffUsFormation::robotsInFormationPtr() {
    return robotsInFormation;
}

} // ai
} // rtt