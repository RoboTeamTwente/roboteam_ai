//
// Created by jessevw on 24.03.20.
//

#include "stp/plays/referee_specific/Halt.h"

#include "stp/evaluations/game_states/HaltGameStateEvaluation.h"
#include "stp/roles/passive/Halt.h"

namespace rtt::ai::stp::play {

    Halt::Halt() : Play() {
        startPlayEvaluation.clear();
        startPlayEvaluation.emplace_back(GlobalEvaluation::HaltGameState);

        keepPlayEvaluation.clear();
        keepPlayEvaluation.emplace_back(GlobalEvaluation::HaltGameState);

        roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
                std::make_unique<role::Halt>(role::Halt("halt_0")),
                std::make_unique<role::Halt>(role::Halt("halt_1")),
                std::make_unique<role::Halt>(role::Halt("halt_2")),
                std::make_unique<role::Halt>(role::Halt("halt_3")),
                std::make_unique<role::Halt>(role::Halt("halt_4")),
                std::make_unique<role::Halt>(role::Halt("halt_5")),
                std::make_unique<role::Halt>(role::Halt("halt_6")),
                std::make_unique<role::Halt>(role::Halt("halt_7")),
                std::make_unique<role::Halt>(role::Halt("halt_8")),
                std::make_unique<role::Halt>(role::Halt("halt_9")),
                std::make_unique<role::Halt>(role::Halt("halt_10"))};
    }

    uint8_t Halt::score(PlayEvaluator &playEvaluator) noexcept {
        /// List of all factors that combined results in an evaluation how good the play is.
        scoring = {{playEvaluator.getGlobalEvaluation(eval::HaltGameState), 1.0}};
        return (lastScore = playEvaluator.calculateScore(scoring)).value(); // DONT TOUCH.
    }

    Dealer::FlagMap Halt::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;

        const std::string roleName = "halt_";
        for (int i = 0; i <= 10; i++) {
            flagMap.insert({roleName + std::to_string(i), {DealerFlagPriority::REQUIRED, {}}});
        }

        return flagMap;
    }

    void Halt::calculateInfoForRoles() noexcept {
//        /*1*/std::vector<Vector2> challengePos = {Vector2(2.9,0), Vector2(2,0.75), Vector2(2,0.55),Vector2(0.2,1.8)}; // works
        /*2*/std::vector<Vector2> challengePos = {Vector2(2.9,-0.175), Vector2(2,-0.175), Vector2(1.5,0),Vector2(0.9,0)}; // works
//        /*3*/std::vector<Vector2> challengePos = {Vector2(2.9,0), Vector2(2,-0.5), Vector2(2,0.5),Vector2(1,0),Vector2(0.2,0)}; // works
//        /*4*/std::vector<Vector2> challengePos = {Vector2(2.9,-0.0), Vector2(1.95,-0.5), Vector2(1.95,-0.0),Vector2(1.95,0.5),Vector2(1.35,0)}; // works
//        /*5*/std::vector<Vector2> challengePos = {Vector2(2.9,0.2), Vector2(2,-0.0), Vector2(2,-0.5),Vector2(2.85,-1.1),Vector2(1.45,-1.75),Vector2(2.85,-1.75)}; // BROKEN shoots too close to goal edge
//        /*6*/std::vector<Vector2> challengePos = {Vector2(2.9,0.0), Vector2(2.4,-1.25), Vector2(2,-0.95),Vector2(2.0,-0.36),Vector2(1.45,1.21),Vector2(0.95,1.55)}; // works
//        /*7*/std::vector<Vector2> challengePos = {Vector2(2.91,0.3), Vector2(2.6,-1.2), Vector2(2,-0.3),Vector2(1.7,0.35),Vector2(2.3,1.3),Vector2(2.8,1.8)}; //BROKEN, drives into defense area
//        /*8*/std::vector<Vector2> challengePos = {Vector2(2.9,0.1), Vector2(2.25,-1.2), Vector2(2.05,-0.8),Vector2(2.05,-0.4),Vector2(2.05,0.4),Vector2(2.05,0.8),Vector2(0.2,-1.8)}; //BROKEN?, shoots into a robot or misses most of the time
//        /*9*/std::vector<Vector2> challengePos = {Vector2(2.9,0.3), Vector2(2.05,-1.05), Vector2(0.8,-1.0),Vector2(1.35,-0.35),Vector2(1.55,0.6),Vector2(2.2,1.2),Vector2(2.8,1.75)}; //broken, drives into defense area
//        /*10*/std::vector<Vector2> challengePos = {Vector2(2.9,0.15), Vector2(2.4,-1.25), Vector2(2,-0.45),Vector2(1.3,-0.0),Vector2(1.85,0.3),Vector2(2.0,0.9),Vector2(1.3,-0.65)};


        interface::Input::drawData(interface::Visual::PATHFINDING, challengePos, Qt::magenta, 0, interface::Drawing::LINES_CONNECTED);
        interface::Input::drawData(interface::Visual::PATHFINDING, challengePos, Qt::red, 0, interface::Drawing::CROSSES
                                   );
    }

    void Halt::calculateInfoForScoredRoles(world::World *) noexcept {}

    const char *Halt::getName() { return "Halt"; }
}  // namespace rtt::ai::stp::play
