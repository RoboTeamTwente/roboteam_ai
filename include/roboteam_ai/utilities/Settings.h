#pragma once

#include <string>

namespace rtt {

class Settings {
   public:
    enum RobotHubMode { UNKNOWN, BASESTATION, SIMULATOR };

    Settings();
    void init(int idOfAI);
    // This function takes directly the values of the settings of Primary AI,
    // and will convert them to settings this AI should have.
    void handleSettingsFromPrimaryAI(bool isYellow, bool isLeft, RobotHubMode mode, std::string visionIp, int visionPort, std::string refereeIp, int refereePort,
                                     std::string robotHubSendIp, int robotHubSendPort);

    bool isPrimaryAI() const;

    int getId() const;
    void setId(int id);

    bool isYellow() const;
    bool setYellow(bool yellow);

    bool isLeft() const;
    void setLeft(bool left);

    RobotHubMode getRobotHubMode() const;
    bool setRobotHubMode(RobotHubMode mode);

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

    static std::string robotHubModeToString(RobotHubMode mode);

   private:
    int id = 0;
    bool yellow = true;
    bool left = true;
    RobotHubMode robotHubMode;

    std::string visionIp;
    int visionPort;
    std::string refereeIp;
    int refereePort;
    std::string robothubSendIp;
    int robothubSendPort;
};

extern Settings SETTINGS;

}  // namespace rtt
