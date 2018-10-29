//
// Created by baris on 22/10/18.
//

#ifndef ROBOTEAM_AI_ISINZONE_H
#define ROBOTEAM_AI_ISINZONE_H

#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/GeometryFieldSize.h"

#include "Condition.h"
#include "../utilities/World.h"

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"

namespace rtt {
    namespace ai {

        class IsInZone : public Condition {
        public:
            explicit IsInZone(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);

            Status Update() override;

        private:
//    ros::NodeHandle n; //TODO: look into this
        };

    } // ai
} //rtt

#endif //ROBOTEAM_AI_ISINZONE_H
