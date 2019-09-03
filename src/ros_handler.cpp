#include "roboteam_world/ros_handler.h"
#include "roboteam_utils/constants.h"
#include "roboteam_proto/messages_robocup_ssl_wrapper.pb.h"
#include "roboteam_proto/messages_robocup_ssl_wrapper_legacy.pb.h"
#include <roboteam_world/net/robocup_ssl_client.h>
#include <sstream>

namespace rtt {

    /// Initiate the world ros_handler
    void RosHandler::init(rtt::WorldBase* _world) {
      KF = new kalmanFilter;
      world = _world;
      pub = new roboteam_proto::Publisher(ROBOTEAM_WORLD_TCP_PUBLISHER);
    }

    void RosHandler::kalmanLoop() {
      roboteam_proto::SSL_WrapperPacket vision_packet;
        roboteam_proto::SSL_Referee ref_packet;


      constexpr int DEFAULT_VISION_PORT = 10006;
      constexpr int DEFAULT_REFEREE_PORT = 10003;

      const std::string SSL_VISION_SOURCE_IP = "224.5.23.2";
      const std::string SSL_REFEREE_SOURCE_IP= "224.5.23.1";

     // const std::string LOCAL_SOURCE_IP = "127.0.0.1";
      //const std::string LOCAL_SOURCE_IP = "224.5.23.3";
      // const std::string LOCAL_SOURCE_IP = "224.5.23.3";

      vision_client = new RoboCupSSLClient(DEFAULT_VISION_PORT, SSL_VISION_SOURCE_IP);
      refbox_client = new RoboCupSSLClient(DEFAULT_REFEREE_PORT, SSL_REFEREE_SOURCE_IP);

      std::cout << "Vision  : " << SSL_VISION_SOURCE_IP  << ":" << DEFAULT_VISION_PORT << std::endl;
        std::cout << "Referee  : " << SSL_REFEREE_SOURCE_IP  << ":" << DEFAULT_REFEREE_PORT << std::endl;

      vision_client->open(false); // boolean blocking
      refbox_client->open(false);

      std::this_thread::sleep_for(std::chrono::microseconds(10000));


      std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(
          std::chrono::system_clock::now().time_since_epoch()
      );

      std::chrono::milliseconds last_call_time = ms;

      bool ok = true;
        while(ok) {

          std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(
              std::chrono::system_clock::now().time_since_epoch()
          );
          std::chrono::milliseconds now_time = ms;

          auto diff = now_time - last_call_time;

          auto timeDiff =std::chrono::milliseconds(10); // 100hz

          if(diff > timeDiff) {

            if (vision_client) {
                while (vision_client->receive(vision_packet)) {
                    if (vision_packet.has_detection()) {
                  detection_callback(vision_packet.detection());
                }

                if (vision_packet.has_geometry()) {
                  std::ostringstream stream;
                  auto geom = vision_packet.geometry();
                  geom.SerializeToOstream(&stream);

                  // Publish the data.
                  pub->send(TOPIC_GEOMETRY, stream.str());
                }
              }
            }

              if (refbox_client) {
                  while (refbox_client->receive(ref_packet)) {
                      std::ostringstream stream;
                      auto geom = ref_packet;
                      geom.SerializeToOstream(&stream);
                      pub->send(TOPIC_REFEREE, stream.str());
                  }
              }

            KF->kalmanUpdate();
            pub->send(TOPIC_WORLD_STATE, KF->getWorldMsg());
            last_call_time = now_time;//last time is re initialized
          }else {
            std::this_thread::sleep_for(timeDiff-diff);
          }
        }
    }

    void RosHandler::setKalman(bool on) {
        kalman=on;
    }


    /// Callback function for /vision_detection in ros_handler
    void RosHandler::detection_callback(roboteam_proto::SSL_DetectionFrame frame) {
        if (kalman) {
          KF->newFrame(frame);
          return;
        }
        world->detection_callback(frame);

        if (auto * fworld = dynamic_cast<FilteredWorld*>(world)) {
            // Filtered world! Special case that shit
            if (auto worldOpt = fworld->consumeMsg()) {
                pub->send(TOPIC_WORLD_STATE, worldOpt->SerializeAsString());

            }
        } else {
            // Generic world approach
            // Where you can never be sure if this message is the new one
            roboteam_proto::World world_msg = world->as_message();
          pub->send(TOPIC_WORLD_STATE, world_msg.SerializeAsString());
        }
    }
}
