#include "roboteam_world/ros_handler.h"
#include "roboteam_utils/constants.h"
#include "messages_robocup_ssl_wrapper.pb.h"
#include "messages_robocup_ssl_wrapper_legacy.pb.h"
#include <roboteam_world/net/robocup_ssl_client.h>
#include <sstream>

namespace rtt {

    /// Initiate the world ros_handler
    void RosHandler::init(rtt::WorldBase* _world) {
      KF = new kalmanFilter;
      world = _world;
      pub = new roboteam_proto::Publisher();
    }


    void RosHandler::getMessages() {



//
//        std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(
//            std::chrono::system_clock::now().time_since_epoch()
//        );
//
//        std::chrono::milliseconds last_call_time = ms;
//
//        bool ok = true;
//        while(ok) {
//
//          std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(
//              std::chrono::system_clock::now().time_since_epoch()
//          );
//          std::chrono::milliseconds now_time = ms;
//          auto diff = now_time - last_call_time;
//
//          auto timeDiff =std::chrono::milliseconds(20);
//
//          if(diff > timeDiff) { // send at 50 hz
//
//
//
//        } else {
//            std::this_thread::sleep_for(timeDiff-diff);
//          }
//
//
//      }


    }


    void RosHandler::kalmanLoop() {
      roboteam_proto::SSL_WrapperPacket vision_packet;


      constexpr int DEFAULT_VISION_PORT = 10006;
      constexpr int DEFAULT_REFEREE_PORT = 10003;

//      const std::string SSL_VISION_SOURCE_IP = "224.5.23.2";
//      const std::string SSL_REFEREE_SOURCE_IP= "224.5.23.1";

     // const std::string LOCAL_SOURCE_IP = "127.0.0.1";
      //const std::string LOCAL_SOURCE_IP = "224.5.23.3";
      const std::string LOCAL_SOURCE_IP = "224.5.23.3";

      vision_client = new RoboCupSSLClient(DEFAULT_VISION_PORT, LOCAL_SOURCE_IP);

      std::cout << "Vision  : " << LOCAL_SOURCE_IP  << ":" << DEFAULT_VISION_PORT << std::endl;

      vision_client->open(false); // boolean blocking


        std::thread thread(&RosHandler::getMessages, this);
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

          auto timeDiff =std::chrono::milliseconds(20);

          if(diff > timeDiff) { // send at 50 hz




            if (vision_client) {

              // Receive current version packets.
              while (vision_client->receive(vision_packet)) {

                // Detection frame.
                if (vision_packet.has_detection()) {
                  detection_callback(vision_packet.detection());
                }

                if (vision_packet.has_geometry()) {

                  std::ostringstream stream;
                  auto geom = vision_packet.geometry();
                  geom.SerializeToOstream(&stream);

                  // Publish the data.
                  pub->send("geometry", stream.str());
                }
              }
            }



            KF->kalmanUpdate();
            pub->send("world_state", KF->getWorldMsg());
            last_call_time = now_time;//last time is re initialized
          }else {
            std::this_thread::sleep_for(timeDiff-diff);
          }
        }
        thread.join();
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
                pub->send("world_state", worldOpt->SerializeAsString());

            }
        } else {
            // Generic world approach
            // Where you can never be sure if this message is the new one
            roboteam_proto::World world_msg = world->as_message();
          pub->send("world_state", world_msg.SerializeAsString());
        }
    }
}
