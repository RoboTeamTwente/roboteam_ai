//
// Created by baris on 6-12-18.
//

#include "Coach.h"
#include "../interface/InterfaceValues.h"
namespace rtt {
namespace ai {
namespace coach {

using dealer = robotDealer::RobotDealer;


int Coach::pickOpponentToCover(int selfID) {
//    dangerfinder::DangerData DangerData = dangerfinder::DangerFinder::instance().getMostRecentData();
//    std::vector<int> dangerList = DangerData.dangerList;
//    for (int &opponentID : dangerList) {
//        if (defencePairs.find(opponentID) == defencePairs.end()) {
//            if (! World::theirBotHasBall(static_cast<unsigned int>(opponentID))) {
//                return opponentID;
//            }
//        }
//        else if (defencePairs[opponentID] == selfID) {
//            return opponentID;
//        }
//    }

    return - 1;
}

std::pair<int, bool> Coach::getRobotClosestToBall() {
    auto closestUs = World::getRobotClosestToPoint(World::get_world().us, World::getBall()->pos);
    auto closestThem = World::getRobotClosestToPoint(World::get_world().them, World::getBall()->pos);

    auto distanceToBallUs = (Vector2(closestUs->pos).dist(Vector2(World::getBall()->pos)));
    auto distanceToBallThem = (Vector2(closestThem->pos).dist(Vector2(World::getBall()->pos)));

    roboteam_msgs::WorldRobot closestRobot;
    bool weAreCloser;
    if (distanceToBallUs < distanceToBallThem) {
        closestRobot = * closestUs;
        weAreCloser = true;
    } else {
        closestRobot = * closestThem;
        weAreCloser = false;
    }

    return std::make_pair(closestRobot.id, weAreCloser);
}

std::shared_ptr<roboteam_msgs::WorldRobot> Coach::getRobotClosestToBall(bool isOurTeam) {
    return World::getRobotClosestToPoint(isOurTeam ? World::get_world().us : World::get_world().them, World::getBall()->pos);
}
Vector2 Coach::getDemoKeeperGetBallPos(Vector2 ballPos){
    return ballPos+Vector2(0.2,0);
}
} //control
} //ai
} //rtt
