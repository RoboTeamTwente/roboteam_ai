//
// Created by rolf on 17-10-18.
//

#ifndef ROBOTEAM_AI_ISINDEFENSEAREA_HPP
#define ROBOTEAM_AI_ISINDEFENSEAREA_HPP

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include  <boost/optional.hpp>
#include "roboteam_utils/Vector2.h"
#include "../utilities/World.h"
#include "Condition.h"
//TODO: Fix node_name() to be correct.
namespace rtt {
    namespace ai {

        class IsInDefenseArea : public ai::Condition {
        public:
            explicit IsInDefenseArea(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);

            Status Update() override;
        };

    }// ai
}// rtt
#endif //ROBOTEAM_AI_ISINDEFENSEAREA_HPP
