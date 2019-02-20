//
// Created by rolf on 18-2-19.
//

#include "DefensiveCoach.h"

/// This is a class that returns the positions we want our defenders to be at for all defenders
namespace rtt{
namespace ai{
namespace coach{

/// returns a vector with robot ID's and pass likelyhood
std::vector<std::pair<int,double>> DefensiveCoach::possibleTheirPassTargets() {
    std::vector<std::pair<int,double>> passTargets;
    //first find if one of their bots has the ball and if so which bot
    int theirBotWithBall=World::whichBotHasBall(false);
    if (theirBotWithBall==-1)
    {
        return passTargets;
    }
    for (auto bot: World::get_world().them){
        if (bot.id!=theirBotWithBall){
            double danger=calculatePassDanger(theirBotWithBall, bot.id);
            passTargets.push_back({bot.id,danger});
        }
    }
    return passTargets;
}

// calculates the danger of a single possible pass
double DefensiveCoach::calculatePassDanger(int passer, int receiver) {
    std::shared_ptr<roboteam_msgs::WorldRobot> passBot=World::getRobotForId(passer,false);
    std::shared_ptr<roboteam_msgs::WorldRobot> receiveBot=World::getRobotForId(receiver,false);
    double danger=0;
    // We count only our own bots as obstacles, as the opponents bots will likely move out of the way for a pass
    for (auto bot: World::get_world().us){

    }
}
}
}
}