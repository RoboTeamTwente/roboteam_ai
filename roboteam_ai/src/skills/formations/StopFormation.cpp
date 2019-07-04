
#include "StopFormation.h"
#include <roboteam_ai/src/world/Field.h>
#include "../../control/Hungarian.h"

namespace rtt {
namespace ai {

std::shared_ptr<std::vector<std::shared_ptr<world::Robot>>> StopFormation::robotsInFormation = nullptr;

StopFormation::StopFormation(std::string name, bt::Blackboard::Ptr blackboard)
        : Formation(name, blackboard) {
    robotsInFormation = std::make_shared<std::vector<std::shared_ptr<world::Robot>>>();
}

Vector2 StopFormation::getFormationPosition() {
    auto field = world::field->get_field();

    auto pp = world::field->getPenaltyPoint(true); // penalty point

    auto defenseAreaLineA = world::field->get_field().left_penalty_line.begin;
    auto defenseAreaLineB = world::field->get_field().left_penalty_line.end;

    // divide the upper and bottom lines of the defense area and store those values.
    auto dTopY = fmax(defenseAreaLineA.y, defenseAreaLineB.y);
    auto dBtmY = fmin(defenseAreaLineA.y, defenseAreaLineB.y);
    auto defAreaHeight = fabs(dTopY - dBtmY);
    double offset = 0.5;

    //failsafe to prevent segfaults
    int amountOfRobots = robotsInFormation->size();
    if (amountOfRobots <= 0) {
        return {};
    } else if (amountOfRobots == 1) {
        return {pp.x + offset, pp.y};
    }

    float factor = defAreaHeight/robotsInFormation->size()-1;

    // lets get us .5m from our defense area for now

    double fh = field.field_width;
    double fw = field.field_length;

    std::vector<std::vector<Vector2>> targetLocations = {
            // middle
            {{pp.x + offset, pp.y}},

            // two def area corners
            {{pp.x + offset, dTopY}, {pp.x + offset, dBtmY}},

            // middle + def area corners
            {{pp.x + offset, pp.y}, {pp.x + offset, dTopY}, {pp.x + offset, dBtmY}},

            // 4 points in front of def area        (noted from top to bottom)
            {{pp.x + offset, dTopY}, {pp.x + offset, dTopY-factor}, {pp.x + offset,  dBtmY + factor}, {pp.x + offset, dBtmY}},

             // 5 points in front of def area (top to bottom
            {{pp.x + offset, dTopY}, {pp.x + offset, dTopY-factor}, {pp.x + offset, pp.y}, {pp.x + offset,  dBtmY + factor}, {pp.x + offset, dBtmY}},

            // 6 points in front of def area (top to bottom)
            {{pp.x, dTopY+(factor*2.0)},
             {pp.x + offset, dTopY},
             {pp.x + offset, dTopY-(factor*2.0)},
             {pp.x + offset, dBtmY + (factor*2.0)},
             {pp.x + offset, dBtmY},
             {pp.x, dBtmY - (factor*2.0)}},

            // 7 points in front of def area
            {{pp.x + offset, dTopY+(defAreaHeight/3)},
             {pp.x + offset, dTopY},
             {pp.x + offset, pp.y},
             {pp.x + offset, dBtmY},
             {pp.x + offset, dBtmY - (defAreaHeight/3)},
             {pp.x, dTopY+ (defAreaHeight/3)},
             {pp.x, dBtmY - (defAreaHeight/3)}},

            // 8 points in front of def area
            {{pp.x + offset, dTopY+(defAreaHeight/3)},
             {pp.x + offset, dTopY},
             {pp.x + offset, dTopY-(defAreaHeight/3)},
             {pp.x + offset, dBtmY + (defAreaHeight/3)},
             {pp.x + offset, dBtmY},
             {pp.x + offset, dBtmY - (defAreaHeight/3)},
             {pp.x, dTopY+ (defAreaHeight/3)},
             {pp.x, dBtmY - (defAreaHeight/3)}}
    };

    std::vector<int> robotIds;
    rtt::HungarianAlgorithm hungarian;
    auto shortestDistances = hungarian.getRobotPositions(robotIds, true, targetLocations.at(amountOfRobots));
    return shortestDistances.at(robot->id);
}

std::shared_ptr<std::vector<world::World::RobotPtr>> StopFormation::robotsInFormationPtr() {
    return robotsInFormation;
}

}
}