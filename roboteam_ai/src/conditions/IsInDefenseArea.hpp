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
#include "../utilities/Condition.h"
#include "../utilities/WorldUtils.hpp"
#include "ros/ros.h"
namespace rtt{
namespace ai{
    bool isWithinDefenseArea(bool ourDefenseArea, Vector2 point);
    bool isWithinDefenseArea(bool ourDefenseArea, Vector2 point, double margin);
/**
 * \class IsInDefenseArea
 * \brief Checks if a ball or robot is in one of the defense area's.
 */
class IsInDefenseArea : public ai::Condition{
public:
    IsInDefenseArea(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    /**
     * @brief Checks if a ball or robot is in one of the defense area's. Uses the blackboard variable 'ourDefenseArea' for side, and 'margin' for the safety margin used.
     * If 'robot' is set to true and 'ROBOT_ID' is set, then it checks if that robot with specific ID is in the defense area instead. Otherwise, it defaults to the Ball.
     * @return Status::Failure if it is not true, Status::Succes if it is true
     */
    Status Update();
};
}// ai
}// rtt
#endif //ROBOTEAM_AI_ISINDEFENSEAREA_HPP
