#include <roboteam_ai/src/analysis/DecisionMaker.h>
#include "KickOffUsFormation.h"

namespace rtt {
namespace ai {

KickOffUsFormation::KickOffUsFormation(std::string name, bt::Blackboard::Ptr blackboard)
        : Formation(name, blackboard) { }

Vector2 KickOffUsFormation::getFormationPosition() {
    std::vector<Vector2> targetLocations;
    std::vector<int> robotIds;
    auto field = world::field->get_field();
    double targetLocationX;

    analysis::DecisionMaker maker;
    analysis::PlayStyle style = maker.getRecommendedPlayStyle();

    int def = style.amountOfDefenders;
    int mid = style.amountOfMidfielders;
    int att = style.amountOfAttackers;

    if ((def+mid+att) != robotsInFormation.size()) { return { }; }

    int count = 0;

    // set up the defensive locations
    targetLocationX = - field.field_length/3;
    for (int i = 0; i < def; i++) {
        double targetLocationY = ((field.field_width/(def+1))*(i+1)) - field.field_width/2;
        targetLocations.push_back({targetLocationX, targetLocationY});

        if (robotsInFormation.size() > count) {
            robotIds.push_back(robotsInFormation.at(count)->id);
            count++;
        }
    }

    // set up midfield locations
    targetLocationX = - field.field_length/5;
    for (int i = 0; i < mid; i++) {
        double targetLocationY = (((field.field_width/(mid+1))*(i+1)) - field.field_width/2) * 0.8;
        targetLocations.push_back({targetLocationX, targetLocationY});
        if (robotsInFormation.size() > count) {
            robotIds.push_back(robotsInFormation.at(count)->id);
            count++;
        }
    }

    // set up offensive locations
    targetLocationX = -field.field_length/16;

    for (int i = 0; i < att; i++) {
        double targetLocationY = (((field.field_width/(att+1))*(i+1)) - field.field_width/2) * 1.5;
        targetLocations.push_back({targetLocationX, targetLocationY});
        if (robotsInFormation.size() > count) {
            robotIds.push_back(robotsInFormation.at(count)->id);
            count++;
        }
    }

    rtt::HungarianAlgorithm hungarian;
    auto shortestDistances = hungarian.getRobotPositions(robotIds, true, targetLocations);
    return shortestDistances.at(robot->id);
}

} // ai
} // rtt