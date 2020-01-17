//
// Created by john on 1/17/20.
//

#ifndef RTT_ROBOTCONTROLLERS_HPP
#define RTT_ROBOTCONTROLLERS_HPP

#include <memory>
#include <control/shotControllers/ShotController.h>
#include <control/ballHandling/BallHandlePosControl.h>

namespace rtt::world_new::robot {

    /**
     * Structure that allows a global state for robot controllers, allowing to persist past a single tick
     */
    class RobotControllers {
        /**
         * These are self-explanatory, for more information about them check their actual classes
         * Taking ownership of any of these unique_ptr's will result in undefined behavior
         */
        std::unique_ptr<ai::control::ShotController> shotController = std::make_unique<ai::control::ShotController>();
        std::unique_ptr<ai::control::NumTreePosControl> numTreePosControl = std::make_unique<ai::control::NumTreePosControl>();
        std::unique_ptr<ai::control::BasicPosControl> basicPosControl = std::make_unique<ai::control::BasicPosControl>();
        std::unique_ptr<ai::control::BallHandlePosControl> ballHandlePosControl = std::make_unique<ai::control::BallHandlePosControl>();

    public:
        [[nodiscard]] std::unique_ptr<ai::control::ShotController> &getShotController() noexcept;

        [[nodiscard]] std::unique_ptr<ai::control::NumTreePosControl> &getNumTreePosController() noexcept;

        [[nodiscard]] std::unique_ptr<ai::control::BasicPosControl> &getBasicPosController() noexcept;

        [[nodiscard]] std::unique_ptr<ai::control::BallHandlePosControl> &getBallHandlePosController() noexcept;

    };

}

#endif //RTT_ROBOTCONTROLLERS_HPP
