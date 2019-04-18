//
// Created by thijs on 21-3-19.
//

#ifndef ROBOTEAM_AI_WORLDMANAGER_H
#define ROBOTEAM_AI_WORLDMANAGER_H

#include "../io/IOManager.h"
#include "../treeinterp/BTFactory.h"
#include "ros/ros.h"
#include "World.h"
#include "Field.h"
#include "FutureWorld.h"
#include "WorldData.h"

namespace rtt {
namespace ai {
namespace world {

class WorldManager {
    private:

        io::IOManager* IOManager;
        double lastWorldTime;
        roboteam_msgs::World worldMsg;
        roboteam_msgs::GeometryData geometryMsg;
        roboteam_msgs::RefereeData refereeMsg;
        roboteam_msgs::RobotFeedback robotFeedbackMsg;

        unsigned char updateROSData();

        void updateReferee();
        void updateWorld();
        void updateGeometry();
        void updateRobotFeedback();
        void updateGameAnalyzer(const WorldData &worldData);

        unsigned char refereeMsgChanged(roboteam_msgs::RefereeData oldR, roboteam_msgs::RefereeData newR);
        unsigned char worldMsgChanged(roboteam_msgs::World oldW, roboteam_msgs::World newW);
        unsigned char geometryMsgChanged(roboteam_msgs::World newG);

    public:
        void setup();
        void loop();
};

}
}
}

#endif //ROBOTEAM_AI_WORLDMANAGER_H
