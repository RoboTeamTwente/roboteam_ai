#ifndef ROBOTEAM_AI_IO_MANAGERRRR_H
#define ROBOTEAM_AI_IO_MANAGERRRR_H

#include <iostream>
#include "constants.h"
#include "World.pb.h"
#include <RobotFeedback.pb.h>
#include <RobotCommand.pb.h>
#include "Referee.pb.h"
#include <DemoRobot.pb.h>
#include <mutex>
#include <Subscriber.h>
#include <messages_robocup_ssl_geometry.pb.h>
#include <GeometryData.pb.h>

namespace rtt {
namespace ai {
class Pause;

namespace io {




class IOManager {
private:

// Map that converts SSL line and arc names to the more clear RoboTeam ones.
  std::map<std::string, std::string> name_map = {
      std::make_pair("TopTouchLine", "top_line"),
      std::make_pair("BottomTouchLine", "bottom_line"),
      std::make_pair("LeftGoalLine", "left_line"),
      std::make_pair("RightGoalLine", "right_line"),
      std::make_pair("HalfwayLine", "half_line"),
      std::make_pair("CenterLine", "center_line"),
      std::make_pair("LeftPenaltyStretch", "left_penalty_line"),
      std::make_pair("RightPenaltyStretch", "right_penalty_line"),

      std::make_pair("LeftFieldLeftPenaltyArc", "top_left_penalty_arc"),
      std::make_pair("LeftFieldRightPenaltyArc", "bottom_left_penalty_arc"),
      std::make_pair("RightFieldLeftPenaltyArc", "top_right_penalty_arc"),
      std::make_pair("RightFieldRightPenaltyArc", "bottom_right_penalty_arc"),

      std::make_pair("LeftFieldLeftPenaltyStretch", "top_left_penalty_stretch"),
      std::make_pair("LeftFieldRightPenaltyStretch", "bottom_left_penalty_stretch"),
      std::make_pair("RightFieldLeftPenaltyStretch", "bottom_right_penalty_stretch"),
      std::make_pair("RightFieldRightPenaltyStretch", "top_right_penalty_stretch"),

      std::make_pair("CenterCircle", "center_circle"),
  };

        roboteam_proto::World worldMsg;
        roboteam_proto::GeometryData geometryMsg;
        roboteam_proto::RobotFeedback robotFeedbackMsg;
        roboteam_proto::RefereeData refDataMsg;
        roboteam_proto::DemoRobot demoInfoMsg;

        roboteam_proto::Subscriber * worldSubscriber;
        void handleWorldState(roboteam_proto::World * world);


  roboteam_proto::Subscriber * geometrySubscriber;
  void handleGeometry(roboteam_proto::SSL_GeometryData * geometryData);


  //        ros::Subscriber worldSubscriber;
//        ros::Subscriber geometrySubscriber;
//        ros::Subscriber roleFeedbackSubscriber;
//        ros::Subscriber refereeSubscriber;
//        ros::Subscriber demoInfoSubscriber;
//
//        ros::Publisher robotCommandPublisher;
        rtt::ai::Pause* pause;

  roboteam_proto::FieldLineSegment convert_geometry_field_line_segment(roboteam_proto::SSL_FieldLineSegment protoLine);
  roboteam_proto::FieldCircularArc convert_geometry_field_Circular_arc(roboteam_proto::SSL_FieldCicularArc protoArc);
  roboteam_proto::GeometryCameraCalibration convert_geometry_camera_calibration(roboteam_proto::SSL_GeometryCameraCalibration protoCal);

    float mm_to_m(float scalar);


    public:
        explicit IOManager(bool subscribe = false, bool advertise = false);
        void publishRobotCommand(roboteam_proto::RobotCommand cmd);

        const roboteam_proto::World &getWorldState();
        const roboteam_proto::GeometryData &getGeometryData();
        const roboteam_proto::RobotFeedback &getRobotFeedback();
        const roboteam_proto::RefereeData &getRefereeData();
        const roboteam_proto::DemoRobot &getDemoInfo();

        static std::mutex worldStateMutex;
        static std::mutex geometryMutex;
        static std::mutex robotFeedbackMutex;
        static std::mutex refereeMutex;
        static std::mutex demoMutex;

};

} // io
} // ai
} // rtt

#endif //ROBOTEAM_AI_IO_MANAGER_H