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

#include "include/roboteam_ai/demo/JoystickDemo.h"
#include "include/roboteam_ai/utilities/Pause.h"
#include "include/roboteam_ai/world/Field.h"
#include "include/roboteam_ai/world/Robot.h"
#include "include/roboteam_ai/utilities/GameStateManager.hpp"
#include "include/roboteam_ai/io/IOManager.h"

namespace rtt {
namespace ai {
namespace io {

std::mutex IOManager::worldStateMutex;
std::mutex IOManager::geometryMutex;
std::mutex IOManager::robotFeedbackMutex;
std::mutex IOManager::refereeMutex;
std::mutex IOManager::demoMutex;

IOManager::IOManager(bool subscribe, bool advertise) {

   if (subscribe) {
       std::cout << "creating an io manager that subscribes";
        worldSubscriber = new roboteam_proto::Subscriber("world_state", &IOManager::handleWorldState, this);
        geometrySubscriber= new roboteam_proto::Subscriber("geometry", &IOManager::handleGeometry, this);
   }


//
//    if (advertise) {
//        // set up advertisement to publish robotcommands
//        robotCommandPublisher(nodeHandle.advertise<roboteam_proto::RobotCommand>(rtt::TOPIC_COMMANDS, 100);
//    }
}

void IOManager::handleWorldState(roboteam_proto::World * world) {
  std::lock_guard<std::mutex> lock(worldStateMutex);
  this->worldMsg = *world;
  world::world->updateWorld(this->worldMsg);
}

void IOManager::handleGeometry(roboteam_proto::SSL_GeometryData * sslData) {
  std::lock_guard<std::mutex> lock(geometryMutex);

  roboteam_proto::SSL_GeometryFieldSize protoSize = sslData->field();

  roboteam_proto::GeometryFieldSize rosSize;

  rosSize.set_field_length(protoSize.field_length()/1000.0);
  rosSize.set_field_width(protoSize.field_width()/1000.0);
  rosSize.set_goal_width(protoSize.goal_width()/1000.0);
  rosSize.set_goal_depth(protoSize.goal_depth()/1000.0);
  rosSize.set_boundary_width(protoSize.boundary_width()/1000.0);

  for (int i = 0; i < protoSize.field_lines_size(); ++i) {
    roboteam_proto::SSL_FieldLineSegment protoLine = protoSize.field_lines(i);

    roboteam_proto::FieldLineSegment * rosLine = rosSize.add_field_lines();
    auto newLine = convert_geometry_field_line_segment(protoLine);
    rosLine->Swap(&newLine);

    if (rosLine->name() == "top_line") {
      rosSize.set_allocated_top_line(rosLine);
    } else if (rosLine->name() == "bottom_line") {
      rosSize.set_allocated_bottom_line(rosLine);
    } else if (rosLine->name() == "left_line") {
      rosSize.set_allocated_left_line(rosLine);
    } else if (rosLine->name() == "right_line") {
      rosSize.set_allocated_right_line(rosLine);
    } else if (rosLine->name() == "half_line") {
      rosSize.set_allocated_half_line(rosLine);
    } else if (rosLine->name() == "center_line") {
      rosSize.set_allocated_center_line(rosLine);
    } else if (rosLine->name() == "left_penalty_line") {
      rosSize.set_allocated_left_penalty_line(rosLine);
    } else if (rosLine->name() == "right_penalty_line") {
      rosSize.set_allocated_right_penalty_line(rosLine);
    }
      // adding rectangle box lines
    else if (rosLine->name() == "top_left_penalty_stretch") {
      rosSize.set_allocated_top_left_penalty_stretch(rosLine);
    } else if (rosLine->name() == "bottom_left_penalty_stretch") {
      rosSize.set_allocated_bottom_left_penalty_stretch(rosLine);
    } else if (rosLine->name() == "top_right_penalty_stretch") {
      rosSize.set_allocated_top_right_penalty_stretch(rosLine);
    } else if (rosLine->name() == "bottom_right_penalty_stretch") {
      rosSize.set_allocated_bottom_right_penalty_stretch(rosLine);
    }
  }

  for (int i= 0; i < protoSize.field_arcs_size(); ++i) {
    roboteam_proto::SSL_FieldCicularArc protoArc = protoSize.field_arcs(i);
    
    roboteam_proto::FieldCircularArc rosArc =  roboteam_proto::FieldCircularArc(* rosSize.add_field_arcs());
    auto newArc = convert_geometry_field_Circular_arc(protoArc);
    rosArc.Swap(&newArc);

    if (rosArc.name() == "top_left_penalty_arc") {
      rosSize.set_allocated_top_left_penalty_arc(&rosArc);
    } else if (rosArc.name() == "bottom_left_penalty_arc") {
      rosSize.set_allocated_bottom_left_penalty_arc(&rosArc);
    } else if (rosArc.name() == "top_right_penalty_arc") {
      rosSize.set_allocated_top_right_penalty_arc(&rosArc);
    } else if (rosArc.name() == "bottom_right_penalty_arc") {
      rosSize.set_allocated_bottom_right_penalty_arc(&rosArc);
    } else if (rosArc.name() == "center_circle") {
      rosSize.set_allocated_center_circle(&rosArc);
    }
  }

  roboteam_proto::GeometryData geom;
  geom.set_allocated_field(&rosSize);

  for (int i = 0; i < sslData->calib_size(); ++i) {
    roboteam_proto::SSL_GeometryCameraCalibration protoCal = sslData->calib().Get(i);
    roboteam_proto::GeometryCameraCalibration rosCal = convert_geometry_camera_calibration(protoCal);
    
    auto newCalib = geom.add_calib();
    newCalib->Swap(&rosCal);
  }


  std::cout << geom.field().field_length() << std::endl;

  this->geometryMsg = geom;
  world::field->set_field(geom.field());
}


const roboteam_proto::World &IOManager::getWorldState() {
    std::lock_guard<std::mutex> lock(worldStateMutex);
    return this->worldMsg;
}

const roboteam_proto::GeometryData &IOManager::getGeometryData() {
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
//    if (! pause->getPause()) {
//        if (demo::JoystickDemo::checkIfDemoSafe(cmd.id)) {
//
//            // the geneva cannot be received from world, so we set it when it gets sent.
//            auto robot(world::world->getRobotForId(cmd.id, true);
//            if (robot) {
//                if (cmd.geneva_state == 3) {
//                    robot->setGenevaState(cmd.geneva_state);
//                }
//                /*
//                 *
//                 * if there is (recent) feedback we should not need to update internal state here
//                 * Otherwise we should. We need only do it when the new state is valid and different.
//                 */
//                if (!robot->genevaStateIsDifferent(cmd.geneva_state) || !robot->genevaStateIsValid(cmd.geneva_state)) {
//                    cmd.geneva_state(robot->getGenevaState();
//                }
//
//              //  if (!Constants::FEEDBACK_ENABLED() || !robot->hasRecentFeedback()) {
//                    robot->setGenevaState(cmd.geneva_state);
//             //   }
//
//                // only kick and chip when geneva is ready
//                cmd.kicker(cmd.kicker && robot->isGenevaReady();
//                cmd.chipper(cmd.chipper && robot->isGenevaReady();
//                cmd.kicker_forced(cmd.kicker_forced && robot->isGenevaReady();
//                cmd.chipper_forced(cmd.chipper_forced && robot->isGenevaReady();
//
//                if (cmd.kicker) {
//                    interface::Input::drawData(interface::Visual::SHOTLINES, {robot->pos}, Qt::green, robot->id, interface::Drawing::CIRCLES, 36, 36, 8);
//                }
//
//                if (cmd.kicker_forced) {
//                    interface::Input::drawData(interface::Visual::SHOTLINES, {robot->pos}, Qt::green, robot->id, interface::Drawing::DOTS, 36, 36, 8);
//                }
//
//
//                if (cmd.chipper) {
//                    interface::Input::drawData(interface::Visual::SHOTLINES, {robot->pos}, Qt::yellow, robot->id, interface::Drawing::CIRCLES, 36, 36, 8);
//                }
//
//                if (cmd.chipper_forced) {
//                    interface::Input::drawData(interface::Visual::SHOTLINES, {robot->pos}, Qt::yellow, robot->id, interface::Drawing::DOTS, 36, 36, 8);
//                }
//
//                robot->setDribblerState(cmd.dribbler);
//            }
//            // sometimes trees are terminated without having a role assigned.
//            // It is then possible that a skill gets terminated with an empty robot: and then the id can be for example -1.
//            if (cmd.id >= 0 && cmd.id < 16) {
//                robotCommandPublisher.publish(cmd);
//            }
//        }
//        else {
//            ROS_ERROR("Joystick demo has the robot taken over ID:   %s", std::to_string(cmd.id).c_str());
//        }
//    }
//    else {
//        ROS_ERROR("HALT!");
//    }
}

const roboteam_proto::DemoRobot &IOManager::getDemoInfo() {
    std::lock_guard<std::mutex> lock(demoMutex);
    return this->demoInfoMsg;
}



roboteam_proto::FieldLineSegment IOManager::convert_geometry_field_line_segment(roboteam_proto::SSL_FieldLineSegment protoLine) {
  roboteam_proto::FieldLineSegment rosLine;

  rosLine.set_name(std::string(name_map[protoLine.name()]));
  rosLine.mutable_begin()->set_x( mm_to_m(protoLine.p1().x()));
  rosLine.mutable_begin()->set_y( mm_to_m(protoLine.p1().y()));
  rosLine.mutable_end()->set_x( mm_to_m(protoLine.p2().x()));
  rosLine.mutable_end()->set_y( mm_to_m(protoLine.p2().y()));
  rosLine.set_thickness(mm_to_m(protoLine.thickness()));

  return rosLine;
}


/**
 * Converts a protoBuf FieldCircularArc to the ROS version.
 */
roboteam_proto::FieldCircularArc IOManager::convert_geometry_field_Circular_arc(roboteam_proto::SSL_FieldCicularArc protoArc) {
  roboteam_proto::FieldCircularArc rosArc;

  rosArc.set_name(std::string(name_map[protoArc.name()]));
  
  rosArc.mutable_center()->set_x(mm_to_m(protoArc.center().x()));
  rosArc.mutable_center()->set_y(mm_to_m(protoArc.center().y()));
  rosArc.set_radius(mm_to_m(protoArc.radius()));
  rosArc.set_a1( protoArc.a1());
  rosArc.set_a2( protoArc.a2());
  rosArc.set_thickness( mm_to_m(protoArc.thickness()));

  return rosArc;
}

roboteam_proto::GeometryCameraCalibration IOManager::convert_geometry_camera_calibration(roboteam_proto::SSL_GeometryCameraCalibration protoCal) {
  roboteam_proto::GeometryCameraCalibration rosCal;

  rosCal.set_camera_id(protoCal.camera_id());
  rosCal.set_focal_length(mm_to_m(protoCal.focal_length()));
  rosCal.set_principal_point_x(mm_to_m(protoCal.principal_point_x()));
  rosCal.set_principal_point_y(mm_to_m(protoCal.principal_point_y()));
  rosCal.set_distortion(protoCal.distortion());
  rosCal.set_q0(protoCal.q0());
  rosCal.set_q1(protoCal.q1());
  rosCal.set_q2(protoCal.q2());
  rosCal.set_q3(protoCal.q3());
  rosCal.set_tx(mm_to_m(protoCal.tx()));
  rosCal.set_ty(mm_to_m(protoCal.ty()));
  rosCal.set_tz(mm_to_m(protoCal.tz()));
  rosCal.set_derived_camera_world_tx(mm_to_m(protoCal.derived_camera_world_tx()));
  rosCal.set_derived_camera_world_ty(mm_to_m(protoCal.derived_camera_world_ty()));
  rosCal.set_derived_camera_world_tz(mm_to_m(protoCal.derived_camera_world_tz()));

  return rosCal;
}


float IOManager::mm_to_m(float scalar) {
  return scalar / 1000;
}


} // io
} // ai
} // rtt



