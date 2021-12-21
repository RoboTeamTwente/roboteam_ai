//
// Created by mrlukasbos on 1-9-19.
//

#ifndef RTT_SETTINGS_H
#define RTT_SETTINGS_H

#include <roboteam_proto/Setting.pb.h>

namespace rtt {

class Settings {
   private:
    int id = 0;
    bool yellow;
    bool left;
    bool serialMode;

    std::string visionIp;
    int visionPort;
    std::string refereeIp;

   public:
    int getId() const;

    void setId(int id);

    bool isYellow() const;

    void setYellow(bool yellow);

    bool isLeft() const;

    void setLeft(bool left);

    bool isSerialMode() const;

    void setSerialMode(bool serialMode);

    const std::string &getVisionIp() const;

    void setVisionIp(const std::string &visionIp);

    int getVisionPort() const;

    void setVisionPort(int visionPort);

    const std::string &getRefereeIp() const;

    void setRefereeIp(const std::string &refereeIp);

    int getRefereePort() const;

    void setRefereePort(int refereePort);

    const std::string &getRobothubSendIp() const;

    void setRobothubSendIp(const std::string &robothubSendIp);

    int getRobothubSendPort() const;

    void setRobothubSendPort(int robothubSendPort);

   private:
    int refereePort;
    std::string robothubSendIp;
    int robothubSendPort;

   public:
    void init(int id);
    proto::Setting toMessage();
};

extern Settings SETTINGS;
}  // namespace rtt
#endif  // RTT_SETTINGS_H
