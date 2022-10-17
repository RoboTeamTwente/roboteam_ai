//
// Created by alexander on 28-03-22.
//

#ifndef RTT_TESTFIXTURE_H
#define RTT_TESTFIXTURE_H

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

/**
 * Plays are included here
 */
#include "stp/plays/defensive/DefendPass.h"
#include "stp/plays/defensive/DefendShot.h"
#include "stp/plays/defensive/KeeperKickBall.h"
#include "stp/plays/offensive/Attack.h"
#include "stp/plays/offensive/AttackingPass.h"
#include "stp/plays/referee_specific/AggressiveStopFormation.h"
#include "stp/plays/referee_specific/BallPlacementThem.h"
#include "stp/plays/referee_specific/BallPlacementUs.h"
#include "stp/plays/referee_specific/DefensiveStopFormation.h"
#include "stp/plays/referee_specific/FreeKickThem.h"
#include "stp/plays/referee_specific/Halt.h"
#include "stp/plays/referee_specific/KickOffThem.h"
#include "stp/plays/referee_specific/KickOffThemPrepare.h"
#include "stp/plays/referee_specific/KickOffUs.h"
#include "stp/plays/referee_specific/KickOffUsPrepare.h"
#include "stp/plays/referee_specific/PenaltyThem.h"
#include "stp/plays/referee_specific/PenaltyThemPrepare.h"
#include "stp/plays/referee_specific/PenaltyUs.h"
#include "stp/plays/referee_specific/PenaltyUsPrepare.h"

class RTT_AI_Tests : public ::testing::Test {
   protected:
    rtt::world::World* world;

    // Vector of all plays
    std::vector<std::unique_ptr<rtt::ai::stp::Play>> plays;

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
        rtt::ai::GameStateManager::forceNewGameState(RefCommand::NORMAL_START, std::nullopt);
        rtt::ai::interface::Output::setUseRefereeCommands(false);

        // Make sure all plays exist as new
        constructPlays();
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

   private:
    /**
     * Creates a vector containing all plays. If this already exists, clear it before adding new plays.
     */
    void constructPlays() {
        plays.clear();

        /// This play is only used for testing purposes, when needed uncomment this play!
        // plays.emplace_back(std::make_unique<rtt::ai::stp::TestPlay>());

        plays.emplace_back(std::make_unique<rtt::ai::stp::play::AttackingPass>());
        plays.emplace_back(std::make_unique<rtt::ai::stp::play::Attack>());
        plays.emplace_back(std::make_unique<rtt::ai::stp::play::Halt>());
        plays.emplace_back(std::make_unique<rtt::ai::stp::play::DefendShot>());
        plays.emplace_back(std::make_unique<rtt::ai::stp::play::DefendPass>());
        plays.emplace_back(std::make_unique<rtt::ai::stp::play::KeeperKickBall>());
        plays.emplace_back(std::make_unique<rtt::ai::stp::play::DefensiveStopFormation>());
        plays.emplace_back(std::make_unique<rtt::ai::stp::play::AggressiveStopFormation>());
        plays.emplace_back(std::make_unique<rtt::ai::stp::play::BallPlacementUs>());
        plays.emplace_back(std::make_unique<rtt::ai::stp::play::BallPlacementThem>());
        // plays.emplace_back(std::make_unique<rtt::ai::stp::play::TimeOut>());
        // plays.emplace_back(std::make_unique<rtt::ai::stp::play::PenaltyThemPrepare>());
        // plays.emplace_back(std::make_unique<rtt::ai::stp::play::PenaltyUsPrepare>());
        // plays.emplace_back(std::make_unique<rtt::ai::stp::play::PenaltyThem>());
        // plays.emplace_back(std::make_unique<rtt::ai::stp::play::PenaltyUs>());
        plays.emplace_back(std::make_unique<rtt::ai::stp::play::KickOffUsPrepare>());
        plays.emplace_back(std::make_unique<rtt::ai::stp::play::KickOffThemPrepare>());
        // plays.emplace_back(std::make_unique<rtt::ai::stp::play::FreeKickThem>());
        plays.emplace_back(std::make_unique<rtt::ai::stp::play::KickOffUs>());
        plays.emplace_back(std::make_unique<rtt::ai::stp::play::KickOffThem>());
        // plays.emplace_back(std::make_unique<rtt::ai::stp::play::GetBallPossession>());
        // plays.emplace_back(std::make_unique<rtt::ai::stp::play::GetBallRisky>());
        // plays.emplace_back(std::make_unique<rtt::ai::stp::play::ReflectKick>());
        // plays.emplace_back(std::make_unique<rtt::ai::stp::play::GenericPass>());

        // Set the pointer to world for all plays
        {
            auto const& [_, world] = rtt::world::World::instance();
            for (auto& play : plays) {
                play->setWorld(world);
            }
        }
    }
};
#endif  // RTT_TESTFIXTURE_H
