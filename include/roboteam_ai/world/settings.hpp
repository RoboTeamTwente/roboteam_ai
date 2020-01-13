//
// Created by john on 12/16/19.
//

#ifndef RTT_SETTINGS_HPP
#define RTT_SETTINGS_HPP

#include <string_view>

#include <roboteam_proto/Setting.pb.h>

namespace rtt::world::settings {
    /**
     * Settings class used for determining settings
     * Used to establish a protobuf message, type is proto::Setting
     */
    class Settings {
        /**
         * Data ID (for protobuf)
         */
        size_t id = 0;

        /**
         * True if our team is yellow
         * False if our team is blue
         */
        bool yellow;

        /**
         * State that indicates whether we are left or right side on the field
         */
        bool left;

        /**
         * Serial mode of the connection
         */
        bool serialMode;

        /**
         * Socket IP for vision camera
         */
        std::string visionIp;

        /**
         * Port for vision camera
         */
        size_t visionPort;

        /**
         * IP that we should connect to for the referee
         */
        std::string refereeIp;

        /**
         * Port that we should connect to for the referee
         */
        size_t refereePort;

        /**
         * IP that we should send data to for roboteam_robothub
         */
        std::string robotHubSendIp;

        /**
         * Port taht we should send data to for roboteam_robothub
         */
        size_t robotHubSendPort;

    public:
        /**
         * Static setting for the Skills, there are too many to refactor right now
         */
        static Settings* settings;

        /**
         * Gets the settings packet ID
         * @return The ID
         */
        [[nodiscard]] size_t getId() const noexcept;

        /**
         * Sets the ID
         * @param id id to set the current ID to
         */
        void setId(size_t id) noexcept;

        /**
         * Gets the current yellow state
         * @return True if our team is yellow, false if not
         */
        [[nodiscard]] bool isYellow() const noexcept;

        /**
         * Sets the current yellow state
         * @param yellow Boolean, indicates whether our team is yellow
         */
        void setYellow(bool yellow) noexcept;

        /**
         * Checks whether we are left or right on the field
         * @return Returns true if our team is on the left
         */
        [[nodiscard]] bool isLeft() const noexcept;

        /**
         * Sets the current left state
         * @param left Should be True if our team is left, false if not
         */
        void setLeft(bool left) noexcept;

        /**
         * Checks whether the current settings instance has serialMode set to true
         * @return serialMode
         */
        [[nodiscard]] bool isSerialMode() const noexcept;

        /**
         * Sets serialMode
         * @param serialMode State to ser serialMode to
         */
        void setSerialMode(bool serialMode) noexcept;

        /**
         * Gets a constant l-value reference to @ref visionIp
         * @return std::string const& to the IP
         */
        [[nodiscard]] const std::string & getVisionIp() const noexcept;

        /**
         * Sets the current vision IP
         * @param ip String to set the ip to
         */
        void setVisionIp(std::string_view ip) noexcept;

        /**
         * Gets the current vision port
         * @return The vision port as integer
         */
        [[nodiscard]] size_t getVisionPort() const noexcept;

        /**
         * Sets the current vision port
         * @param visionPort The vision port as integer
         */
        void setVisionPort(size_t visionPort) noexcept;

        /**
         * Similar to getVisionIp except for the referee Ip
         * @return The IP of the vision port
         */
        [[nodiscard]] const std::string& getRefereeIp() const noexcept;

        /**
         * Sets the referee IP
         * @param ip IP to set the refereeIp to
         */
        void setRefereeIp(std::string_view ip) noexcept;

        /**
         * Gets the referee port
         * @return Integer representation of the referee port
         */
        [[nodiscard]] size_t getRefereePort() const noexcept;

        /**
         * Sets the referee port
         * @param port Port to set refereePort to
         */
        void setRefereePort(size_t port) noexcept;

        /**
         * Gets the robotHub IP
         * @return const reference to the IP
         */
        [[nodiscard]] const std::string& getRobothubSendIp() const noexcept;

        /**
         * Sets the robothub ip
         * @param ip Ip to set the robothub ip to
         */
        void setRobothubSendIp(std::string_view ip) noexcept;

        /**
         * Gets the robothub connection port
         * @return Integer representation of the port
         */
        [[nodiscard]] size_t getRobothubSendPort() const noexcept;

        /**
         * Sets the robothub port
         * @param port Integer representation of the port
         */
        void setRobothubSendPort(size_t port) noexcept;

        /**
         * Initializes the settings, default initializes/constructs all fields
         * @param id id to set this->id to after initialization
         */
        void init(size_t id) noexcept;

        /**
         * Converts the current settings to a proto::Setting object
         * @return Returns a new proto::Setting object
         */
        [[nodiscard]] proto::Setting toMessage() const noexcept;
    };

} // namespace rtt::world::settings


#endif //RTT_SETTINGS_HPP
