//
// Created by mrlukasbos on 5-3-19.
//

#include "analysis/AnalysisReport.h"

namespace rtt::ai::analysis {

    RobotDanger AnalysisReport::getRobotDangerForId(int id, bool ourTeam) {
        auto robotDangers = ourTeam ? ourRobotsSortedOnDanger : theirRobotSortedOnDanger;

        if (!robotDangers.empty()) {
            for (auto const &dangerPair : robotDangers) {
                if (dangerPair.first->id == id) {
                    return dangerPair.second;
                }
            }
        }

        return {};
    }
}  // namespace rtt::ai::analysis
