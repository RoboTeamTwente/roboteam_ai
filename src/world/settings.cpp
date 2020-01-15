//
// Created by john on 12/16/19.
//

#include "roboteam_world/world/settings.hpp"

namespace rtt::world::settings {
    size_t Settings::getId() const noexcept {
        return id;
    }

    void Settings::setId(size_t _id) noexcept {
        this->id = _id;
    }

    bool Settings::isYellow() const noexcept {
        return yellow;
    }

    void Settings::setYellow(bool _yellow) noexcept {
        this->yellow = _yellow;
    }

    bool Settings::isLeft() const noexcept {
        return left;
    }

    void Settings::setLeft(bool _left) noexcept {
        this->left = _left;
    }

    bool Settings::isSerialMode() const noexcept {
        return serialMode;
    }

    void Settings::setSerialMode(bool _serialMode) noexcept {
        this->serialMode = _serialMode;
    }

    const std::string &Settings::getVisionIp() const noexcept {
        return visionIp;
    }

    void Settings::setVisionIp(std::string_view ip) noexcept {
        this->visionIp = ip;
    }

    size_t Settings::getVisionPort() const noexcept {
        return visionPort;
    }

    void Settings::setVisionPort(size_t _visionPort) noexcept {
        this->visionPort = _visionPort;
    }

    const std::string &Settings::getRefereeIp() const noexcept {
        return refereeIp;
    }

    void Settings::setRefereeIp(std::string_view ip) noexcept {
        this->refereeIp = ip;
    }

    size_t Settings::getRefereePort() const noexcept {
        return refereePort;
    }

    void Settings::setRefereePort(size_t port) noexcept {
        this->refereePort = visionPort;
    }

    const std::string &Settings::getRobothubSendIp() const noexcept {
        return robotHubSendIp;
    }

    void Settings::setRobothubSendIp(std::string_view ip) noexcept {
        this->robotHubSendIp = ip;
    }

    size_t Settings::getRobothubSendPort() const noexcept {
        return robotHubSendPort;
    }

    void Settings::setRobothubSendPort(size_t port) noexcept {
        this->robotHubSendPort = visionPort;
    }

    void Settings::init(size_t _id) noexcept {
        setId(_id);
    }

    proto::Setting Settings::toMessage() const noexcept {
        auto setting = proto::Setting();
        setting.set_id(getId());
        setting.set_isleft(isLeft());
        setting.set_isyellow(isYellow());
        setting.set_serialmode(isSerialMode());
        setting.set_refereeip(getRefereeIp());
        setting.set_refereeport(getRefereePort());
        setting.set_visionip(getVisionIp());
        setting.set_visionport(getVisionPort());
        setting.set_robothubsendip(getRobothubSendIp());
        setting.set_robothubsendport(getRobothubSendPort());
        return setting;
    }

}