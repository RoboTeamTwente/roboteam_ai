#include <analysis/DecisionMaker.h>
#include <analysis/GameAnalyzer.h>
#include <world/Field.h>
#include "skills/formations/KickOffThemFormation.h"
#include "control/Hungarian.h"

namespace rtt {
namespace ai {

    std::shared_ptr<std::vector<std::shared_ptr<world::Robot>>> KickOffThemFormation::robotsInFormation = nullptr;

KickOffThemFormation::KickOffThemFormation(std::string name, bt::Blackboard::Ptr blackboard)
        : Formation(name, blackboard) {
    robotsInFormation = std::make_shared<std::vector<std::shared_ptr<world::Robot>>>();
}

    Vector2 KickOffThemFormation::getFormationPosition() {
        std::vector<int> robotIds;
        auto fieldMsg = field->get_field();
        double fh = fieldMsg.get(FIELD_WIDTH);
        double fw = fieldMsg.get(FIELD_LENGTH);

        std::vector<std::vector<Vector2>> locations = {
                {{-0.8,0}},
                {{-0.8,0}, {-0.4, fh/5}, {-0.4, -fh/5}},
                {{-0.8,0}, {-0.4, fh/5}, {-0.4, -fh/5}},
                {{-0.8,0}, {-0.4, fh/5}, {-0.4, -fh/5}, {-fw/6, 0}},
                {{-0.8,0}, {-0.4, fh/5}, {-0.4, -fh/5}, {-fw/6, -fh/4}, {-fw/6,  fh/4}},
                {{-0.8,0}, {-0.4, fh/5}, {-0.4, -fh/5}, {-fw/6, -fh/4}, {-fw/6,  fh/4}, {-fw/7,  0}},
                {{-0.8,0}, {-0.4, fh/5}, {-0.4, -fh/5}, {-fw/6, -fh/4}, {-fw/6,  fh/4}, {-fw/7,  0}, {-fw/3, 0}},
                {{-0.8,0}, {-0.4, fh/5}, {-0.4, -fh/5}, {-fw/6, -fh/4}, {-fw/6,  fh/4}, {-fw/7,  0}, {-fw/3, -fh/6}, {-fw/3, fh/6}}
        };

        for (auto const &robot : * robotsInFormation) {
            robotIds.push_back(robot->id);
        }

        rtt::HungarianAlgorithm hungarian;
        auto shortestDistances = hungarian.getRobotPositions(robotIds, true, locations[robotsInFormation->size()-1]);
        return shortestDistances.at(robot->id);
    }

std::shared_ptr<std::vector<shared_ptr<world::Robot>>> KickOffThemFormation::robotsInFormationPtr() {
    return robotsInFormation;
}

} // ai
} // rtt