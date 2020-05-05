//
// Created by john on 4/30/20.
//

#include <include/roboteam_ai/stp/invariants/BallCloseToUsInvariant.h>
#include <include/roboteam_ai/stp/invariants/BallGotShotInvariant.h>
#include <include/roboteam_ai/stp/invariants/BallIsFreeInvariant.h>
#include <include/roboteam_ai/stp/invariants/BallMovesSlowInvariant.h>
#include <include/roboteam_ai/stp/invariants/BallOnOurSideInvariant.h>
#include <include/roboteam_ai/stp/invariants/DistanceFromBallInvariant.h>
#include <include/roboteam_ai/stp/invariants/FreedomOfRobotsInvariant.h>
#include <include/roboteam_ai/stp/invariants/GoalVisionInvariant.h>
#include <include/roboteam_ai/stp/invariants/WeHaveBallInvariant.h>
#include <include/roboteam_ai/stp/invariants/WeHaveMajorityInvariant.h>
#include <include/roboteam_ai/stp/invariants/game_states/BallPlacementThemGameStateInvariant.h>
#include <include/roboteam_ai/stp/invariants/game_states/BallPlacementUsGameStateInvariant.h>
#include <include/roboteam_ai/stp/invariants/game_states/FreeKickThemGameStateInvariant.h>
#include <include/roboteam_ai/stp/invariants/game_states/FreeKickUsGameStateInvariant.h>
#include <include/roboteam_ai/stp/invariants/game_states/HaltGameStateInvariant.h>
#include <include/roboteam_ai/stp/invariants/game_states/KickOffThemGameStateInvariant.h>
#include <include/roboteam_ai/stp/invariants/game_states/KickOffThemPrepareGameStateInvariant.h>
#include <include/roboteam_ai/stp/invariants/game_states/KickOffUsGameStateInvariant.h>
#include <include/roboteam_ai/stp/invariants/game_states/KickOffUsPrepareGameStateInvariant.h>
#include <include/roboteam_ai/stp/invariants/game_states/NormalPlayGameStateInvariant.h>
#include <include/roboteam_ai/stp/invariants/game_states/PenaltyThemGameStateInvariant.h>
#include <include/roboteam_ai/stp/invariants/game_states/PenaltyThemPrepareGameStateInvariant.h>
#include <include/roboteam_ai/stp/invariants/game_states/PenaltyUsGameStateInvariant.h>
#include <include/roboteam_ai/stp/invariants/game_states/PenaltyUsPrepareGameStateInvariant.h>
#include <include/roboteam_ai/stp/invariants/game_states/StopGameStateInvariant.h>
#include <include/roboteam_ai/stp/invariants/game_states/TimeOutGameStateInvariant.h>

#include "include/roboteam_ai/interface/widgets/InvariantsWidget.hpp"

#include <QScrollBar>

namespace rtt::ai::interface {
    namespace inv = stp::invariant;
    InvariantsWidget::InvariantsWidget(QWidget* parent) : QTextEdit(parent) {
        this->setReadOnly(true);

        /**
         * Normal invariants
         */
        invariants["Ball close to us"] = std::make_unique<inv::BallCloseToUsInvariant>();
        invariants["Ball got shot"] = std::make_unique<inv::BallGotShotInvariant>();
        invariants["Ball is free"] = std::make_unique<inv::BallIsFreeInvariant>();
        invariants["Ball moves slow"] = std::make_unique<inv::BallMovesSlowInvariant>();
        invariants["Ball on our side"] = std::make_unique<inv::BallOnOurSideInvariant>();
        invariants["Distance from ball"] = std::make_unique<inv::DistanceFromBallInvariant>();
        invariants["Freedom of robots"] = std::make_unique<inv::FreedomOfRobotsInvariant>();
        invariants["Goal vision"] = std::make_unique<inv::GoalVisionInvariant>();
        invariants["We have ball"] = std::make_unique<inv::WeHaveBallInvariant>();
        invariants["We have majority"] = std::make_unique<inv::WeHaveMajorityInvariant>();

        /**
         * Game State invariants
         */
        invariants["gs::Ball placement them"] = std::make_unique<inv::BallPlacementThemGameStateInvariant>();
        invariants["gs::Ball placement us"] = std::make_unique<inv::BallPlacementUsGameStateInvariant>();
        invariants["gs::Free kick them"] = std::make_unique<inv::FreeKickThemGameStateInvariant>();
        invariants["gs::Free kick us"] = std::make_unique<inv::FreeKickUsGameStateInvariant>();
        invariants["gs::Halt"] = std::make_unique<inv::HaltGameStateInvariant>();
        invariants["gs::Kick Off Them"] = std::make_unique<inv::KickOffThemGameStateInvariant>();
        invariants["gs::Kick Off Them Prepare"] = std::make_unique<inv::KickOffThemPrepareGameStateInvariant>();
        invariants["gs::Kick Off Us"] = std::make_unique<inv::KickOffUsGameStateInvariant>();
        invariants["gs::Kick Off Us Prepare"] = std::make_unique<inv::KickOffUsPrepareGameStateInvariant>();
        invariants["gs::Normal Play"] = std::make_unique<inv::NormalPlayGameStateInvariant>();
        invariants["gs::Penalty Them"] = std::make_unique<inv::PenaltyThemGameStateInvariant>();
        invariants["gs::Penalty Them Prepare"] = std::make_unique<inv::PenaltyThemPrepareGameStateInvariant>();
        invariants["gs::Penalty Us"] = std::make_unique<inv::PenaltyUsGameStateInvariant>();
        invariants["gs::Penalty Us Prepare"] = std::make_unique<inv::PenaltyUsPrepareGameStateInvariant>();
        invariants["gs::Stop"] = std::make_unique<inv::StopGameStateInvariant>();
        invariants["gs::Time out"] = std::make_unique<inv::TimeOutGameStateInvariant>();
    }

    void InvariantsWidget::updateInvariants() {
        clear();
        QString result = "";
        auto world = world_new::World::instance()->getWorld().value();
        auto field = world_new::World::instance()->getField().value();
        for (auto const& [name, inv] : invariants) {
            result += name.c_str();
            result += " -> ";
            result += (inv->checkInvariant(world, &field) ? "true" : "false");
            result += "<br>";
        }
        auto sliderPos = verticalScrollBar()->sliderPosition();
        setText(result);
        verticalScrollBar()->setSliderPosition(sliderPos);
    }
}