//
// Created by mrlukasbos on 5-3-19.
//

#include "AnalysisReport.h"

namespace rtt {
namespace ai {
namespace analysis {

RobotDanger AnalysisReport::getRobotDangerForId(int id, bool ourTeam) {
    auto robotDangers = ourTeam ? ourRobotsSortedOnDanger : theirRobotSortedOnDanger;

    if (!robotDangers.empty()) {
        for (auto const &dangerPair : robotDangers) {
            if (dangerPair.first.id == id) {
                return dangerPair.second;
            }
        }
    }

    return {};
}
} // analysis
} // ai
} // rtt
