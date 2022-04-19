//
// Created by alexander on 28-03-22.
//
#include <gtest/gtest.h>
#include <interface/api/Output.h>
#include <utilities/Constants.h>
#include <utilities/Settings.h>

#include <QtWidgets/QApplication>
#include <iostream>

#include "helpers/FieldHelper.h"
#include "helpers/WorldHelper.h"
#include "utilities/GameStateManager.hpp"
#include "utilities/Settings.h"
#include "world/World.hpp"

#ifndef RTT_TESTFIXTURE_H
#define RTT_TESTFIXTURE_H

class RTT_AI_Tests : public ::testing::Test {
   protected:
    rtt::world::World* world;

    /// This function is called before each test is run, ensuring the default environment for each test
    void SetUp() override {
        // Set SETTINGS to new settings class, defaulting to us being left and yellow
        rtt::SETTINGS = rtt::Settings();

        // Set these fields just like they are set in roboteam_ai.cpp
        rtt::SETTINGS.setRobotHubMode(rtt::Settings::RobotHubMode::SIMULATOR);
        rtt::SETTINGS.setVisionIp("127.0.0.1");
        rtt::SETTINGS.setVisionPort(10006);
        rtt::SETTINGS.setRefereeIp("224.5.23.1");
        rtt::SETTINGS.setRefereePort(10003);
        rtt::SETTINGS.setRobothubSendIp("127.0.0.1");
        rtt::SETTINGS.setRobothubSendPort(20011);

        // Set other variables to their default values
        rtt::ai::interface::Output::setKeeperId(-1);
        rtt::ai::GameStateManager::forceNewGameState(RefCommand::HALT, std::nullopt);
        rtt::ai::interface::Output::setInterfaceGameState(rtt::ai::GameState("halt_strategy", "default"));
        rtt::ai::interface::Output::setUseRefereeCommands(false);
    }

    /**
     * Generates a random world with robots and a field (defaults to standard 11v11 with ball on 12x9 field)
     * @param amountYellow the amount of yellow robots to be put on the field
     * @param amountBlue the amount of blue robots to be put on the field
     * @param withBall bool indicating whether there should be a ball on the field
     * @param field optional field which can be used to generate a world with a non-standard field
     * @return pointer to world containing the relevant information
     */
    static rtt::world::World* generateWorld(int amountYellow = 11, int amountBlue = 11, bool withBall = true, std::optional<proto::SSL_GeometryFieldSize> field = std::nullopt) {
        if (!field) field = testhelpers::FieldHelper::generateField();
        auto protoWorld = testhelpers::WorldHelper::getWorldMsg(amountYellow, amountBlue, withBall, field.value());
        auto const& [_, world] = rtt::world::World::instance();
        world->updateWorld(protoWorld);
        world->updateField(field.value());
        return world;
    }
};
#endif  // RTT_TESTFIXTURE_H
