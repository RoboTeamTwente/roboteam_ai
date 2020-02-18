#pragma once

#include "bt/Node.h"
#include "roboteam_utils/Angle.h"
#include "roboteam_utils/Vector2.h"
#include <world_new/World.hpp>

namespace bt {

class Leaf : public Node {
   public:
    Leaf() = default;
    Leaf(std::string name, Blackboard::Ptr blackboard);
    std::string name;

   protected:

    /** Retrieves the robot using the ROLE set in the blackboard. Returns an empty pointer if ROLE is not set
     * @param properties : Blackboard properties
     * @return an (empty) optional pointer to a RobotView
     */
    std::optional<rtt::world_new::view::RobotView> getRobotFromProperties(const bt::Blackboard::Ptr& properties);

    /** Resets the robotId back to -1
     * @param status : Status with which it terminates
     */
    void terminate(Status status) override;

    /** Sets the robot using robotId */
    void updateRobot();

    std::optional<rtt::world_new::view::RobotView> robot;
    std::optional<rtt::world_new::view::BallView> ball;
    int robotId = -1;
};

}  // namespace bt
