//
// Created by john on 12/16/19.
//

#ifndef RTT_ROBOT_HPP
#define RTT_ROBOT_HPP

#include <roboteam_proto/RobotFeedback.pb.h>
#include "roboteam_utils/Angle.h"

#include "roboteam_proto/WorldRobot.pb.h"


#include "control/shotControllers/ShotController.h"
#include "control/numTrees/NumTreePosControl.h"
#include "control/BasicPosControl.h"
#include "control/ballHandling/BallHandlePosControl.h"
#include "team.hpp"


namespace rtt::world_new::robot {

    /**
     * Geneva driver gone
     * 30 / 50 watt motor flag
     *
     * robot still changes:
     *  battery
     *  spinner speed
     */
    class Robot {
    private:
        uint32_t id;
        team::Team team;
        uint8_t genevaState;
        uint8_t previousGenevaState{};

        Vector2 pos;
        Vector2 vel;
        Angle angle;
        Vector2 pidPreviousVel;

        double distanceToBall;
        bool iHaveBall{};
        unsigned long lastUpdatedWorldNumber = 0;

        double angularVelocity;
        double timeGenevaChanged{};
        constexpr static double timeToChangeOneGenevaState{0.2};
        bool workingGeneva;
        bool batteryLow{false};

        unsigned char dribblerState = 0;
        unsigned char previousDribblerState = 0;

        double timeDribblerChanged = 0;
        constexpr static double timeToChangeOneDribblerLevel = 0.18;
        bool workingDribbler;
        bool workingBallSensor;

        std::unique_ptr<ai::control::ShotController> shotController{};
        std::unique_ptr<ai::control::NumTreePosControl> numTreePosControl{};
        std::unique_ptr<ai::control::BasicPosControl> basicPosControl{};
        std::unique_ptr<ai::control::BallHandlePosControl> ballHandlePosControl{};

    private:
        void updateFromFeedback(proto::RobotFeedback &feedback) noexcept;

        void setId(uint32_t id) noexcept;

        void setTeam(team::Team team) noexcept;

        void setGenevaState(uint8_t genevaState) noexcept;

        void setPreviousGenevaState(uint8_t previousGenevaState) noexcept;

        void setPos(const Vector2 &pos) noexcept;

        void setVel(const Vector2 &vel) noexcept;

        void setAngle(const Angle &angle) noexcept;

        void setAngularVelocity(double angularVelocity) noexcept;

        void setTimeGenevaChanged(double timeGenevaChanged) noexcept;

        void setWorkingGeneva(bool workingGeneva) noexcept;

        void setBatteryLow(bool batteryLow) noexcept;

        void setDribblerState(unsigned char dribblerState) noexcept;

        void setPreviousDribblerState(unsigned char previousDribblerState) noexcept;

        void setTimeDribblerChanged(double timeDribblerChanged) noexcept;

        void setWorkingDribbler(bool workingDribbler) noexcept;

        void setWorkingBallSensor(bool workingBallSensor) noexcept;

        void resetShotController() noexcept;

        void resetNumTreePosControl() noexcept;

        void resetBasicPosControl() noexcept;

        void resetBallHandlePosControl() noexcept;

        void setPidPreviousVel(const Vector2 &pidPreviousVel) noexcept;

        void setDistanceToBall(double distanceToBall) noexcept;

        void setIHaveBall(bool iHaveBall) noexcept;

        void setLastUpdatedWorldNumber(unsigned long lastUpdatedWorldNumber) noexcept;

    public:
        [[nodiscard]] uint32_t getId() const noexcept;


        [[nodiscard]] team::Team getTeam() const noexcept;


        [[nodiscard]] uint8_t getGenevaState() const noexcept;


        [[nodiscard]] uint8_t getPreviousGenevaState() const noexcept;


        [[nodiscard]] const Vector2 &getPos() const noexcept;


        [[nodiscard]] const Vector2 &getVel() const noexcept;


        [[nodiscard]] const Angle &getAngle() const noexcept;


        [[nodiscard]] double getAngularVelocity() const noexcept;


        [[nodiscard]] double getTimeGenevaChanged() const noexcept;


        [[nodiscard]] bool isWorkingGeneva() const noexcept;


        [[nodiscard]] bool isBatteryLow() const noexcept;


        [[nodiscard]] unsigned char getDribblerState() const noexcept;


        [[nodiscard]] unsigned char getPreviousDribblerState() const noexcept;


        [[nodiscard]] double getTimeDribblerChanged() const noexcept;


        [[nodiscard]] bool isWorkingDribbler() const noexcept;


        [[nodiscard]] bool isWorkingBallSensor() const noexcept;


        [[nodiscard]] ai::control::ShotController *getShotController() const noexcept;

        [[nodiscard]] ai::control::NumTreePosControl *getNumTreePosControl() const noexcept;

        [[nodiscard]] ai::control::BasicPosControl *getBasicPosControl() const noexcept;

        [[nodiscard]] ai::control::BallHandlePosControl *getBallHandlePosControl() const noexcept;

        [[nodiscard]] const Vector2 &getPidPreviousVel() const noexcept;


        [[nodiscard]] double getDistanceToBall() const noexcept;


        [[nodiscard]] bool isIHaveBall() const noexcept;


        [[nodiscard]] unsigned long getLastUpdatedWorldNumber() const noexcept;

    public:
        explicit Robot(std::unordered_map<uint8_t, proto::RobotFeedback> &feedback, const proto::WorldRobot &copy,
                       team::Team team = team::invalid,
                       unsigned char genevaState = 3, unsigned char dribblerState = 0, unsigned long worldNumber = 0);

        Robot &operator=(Robot &) = delete;

        Robot(Robot const &) = delete;

        Robot &operator=(Robot &&) = default;

        Robot(Robot &&) = default;

    };
} // namespace rtt::world::robot


#endif //RTT_ROBOT_HPP
