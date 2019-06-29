//
// Created by thijs on 15-5-19.
//

#ifndef ROBOTEAM_AI_BALLPLACEMENTWITHINTERFACE_H
#define ROBOTEAM_AI_BALLPLACEMENTWITHINTERFACE_H

#include <roboteam_ai/src/control/ballHandling/BallHandlePosControl.h>
#include "../io/IOManager.h"

namespace rtt {
namespace ai {

class BallPlacementWithInterface {
    public:
        explicit BallPlacementWithInterface() = default;
        void onUpdate();

    private:
        std::shared_ptr<world::Robot> robot;
        control::BallHandlePosControl ballHandlePosControl;
        Vector2 previousTargetPos = Vector2();

        io::IOManager ioManager = io::IOManager(false, true);
        void limitRobotCommand();
        void publishRobotCommand();
        void refreshRobotCommand();
        std::string node_name();
        roboteam_msgs::RobotCommand command;
};

}
}

#endif //ROBOTEAM_AI_BALLPLACEMENTWITHINTERFACE_H
