#include <roboteam_ai/src/analysis/DecisionMaker.h>
#include <roboteam_ai/src/analysis/GameAnalyzer.h>
#include "KickOffThemFormation.h"

namespace rtt {
namespace ai {

    std::shared_ptr<vector<std::shared_ptr<world::Robot>>> KickOffThemFormation::robotsInFormation = nullptr;

KickOffThemFormation::KickOffThemFormation(std::string name, bt::Blackboard::Ptr blackboard)
        : Formation(name, blackboard) {
    robotsInFormation = std::make_shared<vector<std::shared_ptr<world::Robot>>>();
}

    Vector2 KickOffThemFormation::getFormationPosition() {
        std::vector<int> robotIds;
        auto field = world::field->get_field();
        double fh = field.field_width;
        double fw = field.field_length;

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

shared_ptr<vector<shared_ptr<world::Robot>>> KickOffThemFormation::robotsInFormationPtr() {
    return robotsInFormation;
}

} // ai
} // rtt