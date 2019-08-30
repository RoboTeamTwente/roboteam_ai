/*
 * Created by mrlukasbos on 19-9-18.
 *
 * This class gives handles for all ROS communication for roboteam_ai.
 * Using this class you don't have to think about callbacks or scoping, or weird ROS parameters.
 */

#include <DemoRobot.pb.h>
#include <RobotFeedback.pb.h>
#include <include/roboteam_ai/io/IOManager.h>
#include <messages_robocup_ssl_geometry.pb.h>
#include <roboteam_utils/constants.h>

#include "include/roboteam_ai/demo/JoystickDemo.h"
#include "include/roboteam_ai/utilities/Pause.h"
#include "include/roboteam_ai/world/Field.h"
#include "include/roboteam_ai/world/Robot.h"
#include "include/roboteam_ai/utilities/GameStateManager.hpp"
#include "include/roboteam_ai/io/IOManager.h"
#include "include/roboteam_ai/interface/api/Input.h"

namespace rtt {
namespace ai {
namespace io {

std::mutex IOManager::worldStateMutex;
std::mutex IOManager::geometryMutex;
std::mutex IOManager::robotFeedbackMutex;
std::mutex IOManager::refereeMutex;
std::mutex IOManager::demoMutex;

IOManager io;

void IOManager::handleWorldState(roboteam_proto::World * world) {
  std::lock_guard<std::mutex> lock(worldStateMutex);
  this->worldMsg = *world;
  world::world->updateWorld(this->worldMsg);
}

void IOManager::handleGeometry(roboteam_proto::SSL_GeometryData * sslData) {
  std::lock_guard<std::mutex> lock(geometryMutex);
  this->geometryMsg = *sslData;

  // protobuf objects are not very long-lasting so convert it into an object which we can store way longer in field
  FieldMessage msg = FieldMessage(sslData->field());
  world::field->set_field(msg);
}


const roboteam_proto::World &IOManager::getWorldState() {
    std::lock_guard<std::mutex> lock(worldStateMutex);
    return this->worldMsg;
}

const roboteam_proto::SSL_GeometryData &IOManager::getGeometryData() {
    std::lock_guard<std::mutex> lock(geometryMutex);
    return this->geometryMsg;
}

const roboteam_proto::RobotFeedback &IOManager::getRobotFeedback() {
//    std::lock_guard<std::mutex> lock(robotFeedbackMutex);
//    return this->robotFeedbackMsg;
}

const roboteam_proto::RefereeData &IOManager::getRefereeData() {
//    std::lock_guard<std::mutex> lock(refereeMutex);
//    return this->refDataMsg;
}

void IOManager::publishRobotCommand(roboteam_proto::RobotCommand cmd) {
    if (! pause->getPause()) {
        if (demo::JoystickDemo::checkIfDemoSafe(cmd.id())) {

            // the geneva cannot be received from world, so we set it when it gets sent.
            auto robot = world::world->getRobotForId(cmd.id(), true);
            if (robot) {
                if (cmd.geneva_state() == 3) {
                    robot->setGenevaState(cmd.geneva_state());
                }
                /*
                 *
                 * if there is (recent) feedback we should not need to update internal state here
                 * Otherwise we should. We need only do it when the new state is valid and different.
                 */
                if (!robot->genevaStateIsDifferent(cmd.geneva_state()) || !robot->genevaStateIsValid(cmd.geneva_state())) {
                    cmd.set_geneva_state(robot->getGenevaState());
                }

              //  if (!Constants::FEEDBACK_ENABLED() || !robot->hasRecentFeedback()) {
                    robot->setGenevaState(cmd.geneva_state());
             //   }

                // only kick and chip when geneva is ready
                cmd.set_kicker(cmd.kicker() && robot->isGenevaReady());
                cmd.set_chipper(cmd.chipper() && robot->isGenevaReady());
                cmd.set_chip_kick_forced(cmd.chip_kick_forced() && robot->isGenevaReady());

                if (cmd.kicker()) {
                    interface::Input::drawData(interface::Visual::SHOTLINES, {robot->pos}, Qt::green, robot->id, interface::Drawing::CIRCLES, 36, 36, 8);
                }

                if (cmd.chip_kick_forced()) {
                    interface::Input::drawData(interface::Visual::SHOTLINES, {robot->pos}, Qt::green, robot->id, interface::Drawing::DOTS, 36, 36, 8);
                }


                if (cmd.chipper()) {
                    interface::Input::drawData(interface::Visual::SHOTLINES, {robot->pos}, Qt::yellow, robot->id, interface::Drawing::CIRCLES, 36, 36, 8);
                }


                robot->setDribblerState(cmd.dribbler());
            }
            // sometimes trees are terminated without having a role assigned.
            // It is then possible that a skill gets terminated with an empty robot: and then the id can be for example -1.
            if (cmd.id() >= 0 && cmd.id() < 16) {
                robotCommandPublisher->send(TOPIC_COMMANDS, cmd.SerializeAsString());
            }
        }
        else {
         //   ROS_ERROR("Joystick demo has the robot taken over ID:   %s", std::to_string(cmd.id).c_str());
        }
    }
    else {
   //     ROS_ERROR("HALT!");
    }
}

const roboteam_proto::DemoRobot &IOManager::getDemoInfo() {
    std::lock_guard<std::mutex> lock(demoMutex);
    return this->demoInfoMsg;
}


void IOManager::init() {
  worldSubscriber = new roboteam_proto::Subscriber(ROBOTEAM_WORLD_TCP_PUBLISHER, TOPIC_WORLD_STATE, &IOManager::handleWorldState, this);
  geometrySubscriber= new roboteam_proto::Subscriber(ROBOTEAM_WORLD_TCP_PUBLISHER, TOPIC_GEOMETRY, &IOManager::handleGeometry, this);


  // set up advertisement to publish robotcommands
  robotCommandPublisher = new roboteam_proto::Publisher(ROBOTEAM_AI_TCP_PUBLISHER);
}

} // io
} // ai
} // rtt



