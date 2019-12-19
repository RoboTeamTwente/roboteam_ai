#include "roboteam_utils/constants.h"
#include "roboteam_proto/messages_robocup_ssl_wrapper.pb.h"
#include <sstream>
#include <roboteam_utils/Timer.h>
#include <roboteam_world/WorldHandler.h>

namespace world {

void WorldHandler::start() {
    init();
    setupSSLClients();

    proto::SSL_WrapperPacket vision_packet;
    proto::SSL_Referee ref_packet;

    roboteam_utils::Timer t;
    t.loop([&]() {
      handleVisionPackets(vision_packet);
      handleRefboxPackets(ref_packet);

      KF->kalmanUpdate();
      world_pub->send(KF->getWorld());
    }, 100);
}

void WorldHandler::init() {
    world = new WorldBase();
    KF = new KalmanFilter;
    world_pub = new proto::Publisher<proto::World>(proto::WORLD_CHANNEL);
    ref_pub = new proto::Publisher<proto::SSL_Referee>(proto::REFEREE_CHANNEL);
    geom_pub = new proto::Publisher<proto::SSL_GeometryData>(proto::GEOMETRY_CHANNEL);
}


void WorldHandler::setupSSLClients() {
    constexpr int DEFAULT_VISION_PORT = 10006;
    constexpr int DEFAULT_REFEREE_PORT = 10003;

    const string SSL_VISION_SOURCE_IP = "224.5.23.2";
    const string SSL_REFEREE_SOURCE_IP = "224.5.23.1";

    vision_client = new RoboCupSSLClient(DEFAULT_VISION_PORT, SSL_VISION_SOURCE_IP);
    refbox_client = new RoboCupSSLClient(DEFAULT_REFEREE_PORT, SSL_REFEREE_SOURCE_IP);

    cout << "Vision  : " << SSL_VISION_SOURCE_IP << ":" << DEFAULT_VISION_PORT << endl;
    cout << "Referee  : " << SSL_REFEREE_SOURCE_IP << ":" << DEFAULT_REFEREE_PORT << endl;

    vision_client->open(false); // boolean blocking
    refbox_client->open(false);
    this_thread::sleep_for(chrono::microseconds(10000));
}

void WorldHandler::handleRefboxPackets(proto::SSL_Referee &ref_packet) const {
    while (refbox_client && refbox_client->receive(ref_packet)) {
        ref_pub->send(ref_packet);
    }
}

void WorldHandler::handleVisionPackets(proto::SSL_WrapperPacket &vision_packet) const {
    while (vision_client && vision_client->receive(vision_packet)) {
        if (vision_packet.has_detection()){
            KF->newFrame(vision_packet.detection());
        }
        if (vision_packet.has_geometry()) {
            geom_pub->send(vision_packet.geometry());
        }
    }
}
}
