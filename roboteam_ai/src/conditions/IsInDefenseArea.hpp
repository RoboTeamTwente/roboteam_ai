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
namespace rtt{
namespace ai{
    bool isWithinDefenseArea(bool ourDefenseArea, Vector2 point);
    bool isWithinDefenseArea(bool ourDefenseArea, Vector2 point, double margin);
/**
 * \class IsInDefenseArea
 * \brief Checks if a ball or robot is in one of the defense area's
 */
//    * Descr: Checks if the ball or robot is in one of the goals
//            * Params:
//    *   - ourDefenseArea:
//    *       Type: Bool
//            *       Descr: Whether to check our defense area, or the opponents'
//    *   - robot:
//    *       Type: Bool
//            *       Descr: Whether this condition applies to the ball or the robot
//    *
//    *   - margin:
//    *       Type: Double
class IsInDefenseArea : public ai::Condition{
public:
    IsInDefenseArea(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update();
};
}// ai
}// rtt
#endif //ROBOTEAM_AI_ISINDEFENSEAREA_HPP
